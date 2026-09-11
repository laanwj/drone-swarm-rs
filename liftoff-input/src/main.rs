//! Liftoff-specific telemetry input.
//!
//! Listens for Liftoff's native UDP telemetry stream (default
//! `127.0.0.1:9001`) and the optional `liftoff-simstate-bridge` UDP
//! stream (default `127.0.0.1:9020`), republishes both onto Zenoh, and
//! generates the workspace-standard CRSF telemetry frames the rest of
//! the stack consumes.
//!
//! No RC channel handling — the virtual joystick lives in `crsf-joystick/`
//! now. This binary is purely Liftoff → Zenoh; for Velocidrone or
//! Uncrashed, run their respective `*-input` crate instead.
use clap::Parser;
use log::{error, info, trace, warn};
use metrics::{Unit, counter, describe_counter};
use metrics_exporter_statsd::StatsdBuilder;
use std::sync::Arc;
use std::time::Duration;
use telemetry_lib::crsf_custom;
use telemetry_lib::crsf_tx;
use telemetry_lib::simstate::{self, BatteryPacket, DamagePacket, SimstatePacket};
use telemetry_lib::telemetry::{self};
use telemetry_lib::topics;
use tokio::net::UdpSocket;
use tokio::sync::{Mutex, Notify};
use zenoh::Config;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Bind address for simulator telemetry UDP.
    #[arg(long, default_value = "127.0.0.1:9001")]
    sim_bind: std::net::SocketAddr,

    /// Bind address for the liftoff-simstate-bridge UDP stream
    /// (per-prop damage + battery telemetry from the BepInEx plugin).
    #[arg(long, default_value = "127.0.0.1:9020")]
    simstate_bind: std::net::SocketAddr,

    /// Zenoh connect endpoint (e.g. tcp/192.168.1.1:7447). Omit for peer discovery.
    #[arg(long)]
    zenoh_connect: Option<String>,

    /// Zenoh mode (peer or client).
    #[arg(long, default_value = "client")]
    zenoh_mode: String,

    /// Zenoh topic prefix.
    #[arg(long, default_value = topics::DEFAULT_PREFIX)]
    zenoh_prefix: String,

    /// Enable metrics reporting to a StatsD collector.
    #[arg(long, default_value_t = false)]
    metrics_statsd: bool,

    /// Destination address for the StatsD collector.
    #[arg(long, default_value = "127.0.0.1:8125")]
    metrics_statsd_destination: std::net::SocketAddr,
}

const TELEMETRY_INTERVAL: Duration = Duration::from_millis(100);
const DAMAGE_HEARTBEAT_INTERVAL: Duration = Duration::from_secs(1);

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    env_logger::init();
    let args = Args::parse();

    info!("Starting liftoff-input");

    if args.metrics_statsd {
        let host = args.metrics_statsd_destination.ip().to_string();
        let recorder = StatsdBuilder::from(&host, args.metrics_statsd_destination.port())
            .build(None)
            .expect("failed to create StatsD recorder");
        metrics::set_global_recorder(recorder)
            .expect("failed to install StatsD recorder");
    }

    describe_counter!(
        "input.telemetry.rx",
        Unit::Count,
        "Telemetry packets received"
    );
    describe_counter!(
        "input.telemetry.tx",
        Unit::Count,
        "CRSF telemetry packets sent"
    );
    describe_counter!(
        "bridge.packet.rx",
        Unit::Count,
        "Incoming telemetry packets from sim"
    );
    describe_counter!(
        "bridge.packet.tx",
        Unit::Count,
        "Telemetry packets published to Zenoh"
    );
    describe_counter!(
        "simstate.damage.rx",
        Unit::Count,
        "Damage UDP packets received from simstate-bridge"
    );
    describe_counter!(
        "simstate.battery.rx",
        Unit::Count,
        "Battery UDP packets received from simstate-bridge"
    );
    describe_counter!(
        "simstate.parse_error",
        Unit::Count,
        "Malformed simstate UDP packets"
    );

    // Zenoh session
    let mut config = Config::default();
    config.insert_json5("mode", &format!(r#""{}""#, args.zenoh_mode))?;
    if let Some(ref endpoint) = args.zenoh_connect {
        config.insert_json5("connect/endpoints", &format!(r#"["{}"]"#, endpoint))?;
    }
    let session = zenoh::open(config).await?;

    let tel_topic = topics::topic(&args.zenoh_prefix, topics::TELEMETRY);
    let crsf_tel_topic = topics::topic(&args.zenoh_prefix, topics::CRSF_TELEMETRY);
    let damage_topic = topics::topic(&args.zenoh_prefix, topics::DAMAGE);
    let battery_topic = topics::topic(&args.zenoh_prefix, topics::BATTERY);

    info!("Subscribing to: {}", tel_topic);
    info!("Publishing on: {}", crsf_tel_topic);
    info!("Publishing on: {} (simstate damage)", damage_topic);
    info!("Publishing on: {} (simstate battery)", battery_topic);

    let crsf_tel_publisher = session.declare_publisher(crsf_tel_topic).await?;
    let tel_subscriber = session.declare_subscriber(&tel_topic).await?;
    let damage_publisher = session.declare_publisher(damage_topic).await?;
    let battery_publisher = session.declare_publisher(battery_topic).await?;

    // Shared latest battery snapshot from the simstate UDP stream. The CRSF
    // generation task reads from here when building BatterySensor + Voltages
    // packets so they carry real current and per-cell voltage instead of
    // just voltage+percent from the sim's standard telemetry.
    let battery_state: Arc<Mutex<Option<BatteryPacket>>> = Arc::new(Mutex::new(None));

    // Shared latest damage snapshot and a Notify to trigger an immediate
    // CRSF damage frame when damage state changes.
    let damage_state: Arc<Mutex<Option<DamagePacket>>> = Arc::new(Mutex::new(None));
    let damage_notify: Arc<Notify> = Arc::new(Notify::new());

    // simstate-bridge UDP listener: forwards raw bytes to the corresponding
    // Zenoh topic and updates the shared battery snapshot.
    let simstate_sock = UdpSocket::bind(args.simstate_bind).await?;
    info!("Bridge: simstate-bridge UDP on {}", args.simstate_bind);
    {
        let battery_state = battery_state.clone();
        let damage_state = damage_state.clone();
        let damage_notify = damage_notify.clone();
        tokio::spawn(async move {
            let mut buf = [0u8; 1024];
            loop {
                match simstate_sock.recv_from(&mut buf).await {
                    Ok((len, _addr)) => {
                        let payload = &buf[..len];
                        match simstate::parse_packet(payload) {
                            Ok(SimstatePacket::Damage(dmg)) => {
                                counter!("simstate.damage.rx").increment(1);
                                // Check if damage values actually changed.
                                let changed = {
                                    let mut guard = damage_state.lock().await;
                                    let changed = guard
                                        .as_ref()
                                        .map(|prev| prev.damage != dmg.damage || prev.flags != dmg.flags)
                                        .unwrap_or(true);
                                    *guard = Some(dmg);
                                    changed
                                };
                                if changed {
                                    damage_notify.notify_one();
                                }
                                if let Err(e) = damage_publisher.put(payload).await {
                                    warn!("Failed to publish damage: {}", e);
                                }
                            }
                            Ok(SimstatePacket::Battery(bat)) => {
                                counter!("simstate.battery.rx").increment(1);
                                *battery_state.lock().await = Some(bat);
                                if let Err(e) = battery_publisher.put(payload).await {
                                    warn!("Failed to publish battery: {}", e);
                                }
                            }
                            Err(e) => {
                                counter!("simstate.parse_error").increment(1);
                                warn!("simstate parse error: {} (len={})", e, len);
                            }
                        }
                    }
                    Err(e) => {
                        error!("simstate UDP recv error: {}", e);
                    }
                }
            }
        });
    }

    // Bridge task: receive sim UDP telemetry and publish to Zenoh
    let bridge_publisher = session.declare_publisher(tel_topic.clone()).await?;
    let sock = UdpSocket::bind(args.sim_bind).await?;
    info!("Bridge: simulator telemetry on {}", args.sim_bind);
    tokio::spawn(async move {
        let mut buf = [0u8; 4096];
        loop {
            match sock.recv_from(&mut buf).await {
                Ok((len, _addr)) => {
                    trace!("rx sim {} bytes", len);
                    counter!("bridge.packet.rx").increment(1);
                    if let Err(e) = bridge_publisher.put(&buf[..len]).await {
                        warn!("Failed to publish sim telemetry: {}", e);
                    } else {
                        counter!("bridge.packet.tx").increment(1);
                    }
                }
                Err(e) => {
                    error!("UDP recv error: {}", e);
                }
            }
        }
    });

    // Telemetry format config
    // We assume default configuration for now
    let config_format = vec![
        "Timestamp".to_string(),
        "Position".to_string(),
        "Attitude".to_string(),
        "Velocity".to_string(),
        "Gyro".to_string(),
        "Input".to_string(),
        "Battery".to_string(),
        "MotorRPM".to_string(),
    ];

    // Task: Receive raw telemetry from bridge, convert to CRSF, publish.
    // Also listens for damage-change notifications to send an immediate
    // damage frame, and includes a 1 Hz damage heartbeat.
    let crsf_tel_pub = crsf_tel_publisher;
    let crsf_battery_state = battery_state.clone();
    let crsf_damage_state = damage_state.clone();
    let crsf_damage_notify = damage_notify.clone();
    let crsf_task = tokio::spawn(async move {
        let mut next_send = tokio::time::Instant::now();
        let mut next_damage_heartbeat = tokio::time::Instant::now();

        /// Publish a single CRSF frame, logging and counting on success.
        async fn send_frame(
            pub_: &zenoh::pubsub::Publisher<'_>,
            pkt: &[u8],
        ) {
            trace!("tx crsf tel {} bytes", pkt.len());
            if let Err(e) = pub_.put(pkt).await {
                warn!("Failed to publish CRSF telem: {}", e);
            } else {
                counter!("input.telemetry.tx").increment(1);
            }
        }

        loop {
            tokio::select! {
                // Normal telemetry path: rate-limited to TELEMETRY_INTERVAL.
                result = tel_subscriber.recv_async() => {
                    match result {
                        Ok(sample) => {
                            let payload = sample.payload().to_bytes();
                            trace!("rx tel {} bytes", payload.len());
                            counter!("input.telemetry.rx").increment(1);
                            let now = tokio::time::Instant::now();
                            if now >= next_send {
                                if let Ok(packet) =
                                    telemetry::parse_packet(&payload, &config_format)
                                {
                                    let bat_snapshot = crsf_battery_state.lock().await.clone();
                                    let crsf_packets =
                                        crsf_tx::generate_crsf_telemetry(&packet, bat_snapshot.as_ref());
                                    for pkt in &crsf_packets {
                                        send_frame(&crsf_tel_pub, pkt).await;
                                    }

                                    // Include damage heartbeat at 1 Hz alongside
                                    // the normal telemetry batch.
                                    if now >= next_damage_heartbeat {
                                        let dmg_snapshot = crsf_damage_state.lock().await.clone();
                                        if let Some(frame) = dmg_snapshot.and_then(|d| crsf_custom::build_damage_packet(&d)) {
                                            send_frame(&crsf_tel_pub, &frame).await;
                                        }
                                        next_damage_heartbeat = now + DAMAGE_HEARTBEAT_INTERVAL;
                                    }

                                    next_send = now + TELEMETRY_INTERVAL;
                                }
                            }
                        }
                        Err(e) => {
                            warn!("Telemetry subscriber error: {}", e);
                            break;
                        }
                    }
                }

                // Immediate damage path: fires when damage state changes.
                _ = crsf_damage_notify.notified() => {
                    let dmg_snapshot = crsf_damage_state.lock().await.clone();
                    if let Some(frame) = dmg_snapshot.and_then(|d| crsf_custom::build_damage_packet(&d)) {
                        send_frame(&crsf_tel_pub, &frame).await;
                    }
                    // Reset heartbeat timer so we don't double-send.
                    next_damage_heartbeat = tokio::time::Instant::now() + DAMAGE_HEARTBEAT_INTERVAL;
                }
            }
        }
    });

    // The CRSF generation task is the last thing keeping us alive — when
    // it exits (telemetry subscriber error / Zenoh shutdown), so do we.
    let _ = crsf_task.await;

    session.close().await?;
    Ok(())
}
