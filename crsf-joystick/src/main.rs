//! Zenoh-driven CRSF → uinput joystick service.
//!
//! Subscribes to two CRSF RC channel topics — manual (`{prefix}/crsf/rc`)
//! and autopilot (`{prefix}/crsf/rc/autopilot`) — muxes between them based
//! on radio presence and the SA switch (channel 7), and applies the
//! winning frame to a virtual `/dev/uinput` controller via
//! [`crsf_joystick::Joystick`].
//!
//! Mux rules:
//! - **Manual frame fresh, SA switch low (channel 7 < `AXIS_MID`)**: manual wins.
//! - **Manual not eligible**: autopilot wins if it is fresh.
//! - **Neither source fresh**: failsafe / neutral wins.
//!
//! "Fresh" is governed by `--source-timeout-ms`. This matches the SA-switch
//! handoff convention used elsewhere in the workspace; the simulator-side
//! bridges (e.g. `liftoff-input`) don't see RC channels at all — they only
//! handle telemetry.
use std::fmt;
use std::time::{Duration, Instant};

use clap::Parser;
use crsf_joystick::{AXIS_MAX, AXIS_MID, Joystick, SAFE_DEFAULT_CHANNELS};
use log::{error, info, trace, warn};
use metrics::{Unit, counter, describe_counter};
use metrics_exporter_tcp::TcpBuilder;
use telemetry_lib::crsf::{self, CrsfPacket};
use telemetry_lib::topics;
use zenoh::Config;

/// Active control source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Source {
    Manual,
    Autopilot,
    Failsafe,
}

impl fmt::Display for Source {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Source::Manual => write!(f, "manual"),
            Source::Autopilot => write!(f, "autopilot"),
            Source::Failsafe => write!(f, "failsafe"),
        }
    }
}

/// Returns true if `t` is present and within `timeout` of now.
fn is_fresh(t: Option<Instant>, timeout: Duration) -> bool {
    t.map(|t| t.elapsed() < timeout).unwrap_or(false)
}

/// Pick the highest-priority source that is currently fresh.
fn select_source(
    last_manual_time: Option<Instant>,
    last_autopilot_time: Option<Instant>,
    last_manual_ch7: u16,
    timeout: Duration,
) -> Source {
    let manual_fresh = is_fresh(last_manual_time, timeout);
    let autopilot_fresh = is_fresh(last_autopilot_time, timeout);

    if manual_fresh && last_manual_ch7 < AXIS_MID {
        Source::Manual
    } else if autopilot_fresh {
        Source::Autopilot
    } else {
        Source::Failsafe
    }
}

/// Compute the next time at which a currently fresh source will become stale.
fn next_deadline(
    now: Instant,
    last_manual_time: Option<Instant>,
    last_autopilot_time: Option<Instant>,
    timeout: Duration,
) -> Instant {
    [last_manual_time, last_autopilot_time]
        .into_iter()
        .filter_map(|t| t.map(|t| t + timeout))
        .filter(|&d| d > now)
        .min()
        .unwrap_or(now + timeout)
}

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Zenoh connect endpoint (e.g. tcp/192.168.1.1:7447). Omit for peer discovery.
    #[arg(long)]
    zenoh_connect: Option<String>,

    /// Zenoh mode (peer or client).
    #[arg(long, default_value = "client")]
    zenoh_mode: String,

    /// Zenoh topic prefix.
    #[arg(long, default_value = topics::DEFAULT_PREFIX)]
    zenoh_prefix: String,

    /// Source timeout in milliseconds. If the selected source has not
    /// produced a valid RC_CHANNELS frame in this time, the output falls back
    /// to autopilot (if fresh) or failsafe defaults.
    #[arg(long, default_value = "100")]
    source_timeout_ms: u64,

    /// Enable metrics reporting using metrics-rs-tcp-exporter.
    #[arg(long, default_value_t = false)]
    metrics_tcp: bool,

    /// Bind address for metrics-rs-tcp-exporter.
    #[arg(long, default_value = "127.0.0.1:5004")]
    metrics_tcp_bind: std::net::SocketAddr,
}

enum Event {
    Frame(Vec<u8>, Source),
    Timeout,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    env_logger::init();
    let args = Args::parse();
    let source_timeout = Duration::from_millis(args.source_timeout_ms);

    info!("Starting crsf-joystick");

    if args.metrics_tcp {
        let builder = TcpBuilder::new().listen_address(args.metrics_tcp_bind);
        builder
            .install()
            .expect("failed to install metrics TCP exporter");
    }

    describe_counter!("joystick.crsf.rx", Unit::Count, "CRSF frames received");
    describe_counter!(
        "joystick.crsf.rx_rc_channels",
        Unit::Count,
        "CRSF RC_CHANNELS frames received"
    );
    describe_counter!(
        "joystick.uinput.update",
        Unit::Count,
        "Updates to virtual input device"
    );
    describe_counter!(
        "joystick.failsafe.enter",
        Unit::Count,
        "Times failsafe state was entered"
    );

    let mut config = Config::default();
    config.insert_json5("mode", &format!(r#""{}""#, args.zenoh_mode))?;
    if let Some(ref endpoint) = args.zenoh_connect {
        config.insert_json5("connect/endpoints", &format!(r#"["{}"]"#, endpoint))?;
    }
    let session = zenoh::open(config).await?;

    let crsf_rc_topic = topics::topic(&args.zenoh_prefix, topics::CRSF_RC);
    let crsf_rc_ap_topic = topics::topic(&args.zenoh_prefix, topics::CRSF_RC_AUTOPILOT);
    info!("Subscribing to: {} (manual)", crsf_rc_topic);
    info!("Subscribing to: {} (autopilot)", crsf_rc_ap_topic);

    let rc_subscriber = session.declare_subscriber(&crsf_rc_topic).await?;
    let rc_ap_subscriber = session.declare_subscriber(&crsf_rc_ap_topic).await?;

    // /dev/uinput requires write permission — typically achieved via udev
    // rule or running as a member of the `input` group.
    let mut joystick = Joystick::new()?;

    // Mux state: track last valid frame from each source.
    let mut last_manual_time: Option<Instant> = None;
    let mut last_autopilot_time: Option<Instant> = None;
    let mut last_manual_channels = SAFE_DEFAULT_CHANNELS;
    let mut last_autopilot_channels = SAFE_DEFAULT_CHANNELS;
    let mut active_source = Source::Failsafe;
    let mut failsafe_deadline = Instant::now() + source_timeout;

    loop {
        let event = tokio::select! {
            result = rc_subscriber.recv_async() => match result {
                Ok(sample) => Event::Frame(sample.payload().to_bytes().to_vec(), Source::Manual),
                Err(e) => { error!("RC subscriber error: {}", e); break; }
            },
            result = rc_ap_subscriber.recv_async() => match result {
                Ok(sample) => Event::Frame(sample.payload().to_bytes().to_vec(), Source::Autopilot),
                Err(e) => { error!("RC autopilot subscriber error: {}", e); break; }
            },
            _ = tokio::time::sleep_until(tokio::time::Instant::from(failsafe_deadline)) => Event::Timeout,
        };

        match event {
            Event::Frame(payload, source) => {
                trace!("rx crsf ({}) {:02x?}", source, &*payload);
                counter!("joystick.crsf.rx").increment(1);

                let Some(CrsfPacket::RcChannelsPacked(channels)) =
                    crsf::parse_packet_check(&payload)
                else {
                    continue;
                };
                counter!("joystick.crsf.rx_rc_channels").increment(1);
                if channels.channels.iter().any(|&c| c > AXIS_MAX) {
                    warn!("Channel out of range: {:?}", channels.channels);
                    continue;
                }

                let now = Instant::now();
                match source {
                    Source::Manual => {
                        last_manual_time = Some(now);
                        last_manual_channels = channels.channels;
                    }
                    Source::Autopilot => {
                        last_autopilot_time = Some(now);
                        last_autopilot_channels = channels.channels;
                    }
                    Source::Failsafe => unreachable!(),
                }
            }
            Event::Timeout => {}
        }

        let selected = select_source(
            last_manual_time,
            last_autopilot_time,
            last_manual_channels[7],
            source_timeout,
        );

        if active_source != selected {
            if selected == Source::Failsafe {
                warn!(
                    "No fresh RC frame for {:?}, entering failsafe",
                    source_timeout
                );
                counter!("joystick.failsafe.enter").increment(1);
            } else {
                info!("RC source switched to {}", selected);
            }
            active_source = selected;
        }

        let channels = match selected {
            Source::Manual => last_manual_channels,
            Source::Autopilot => last_autopilot_channels,
            Source::Failsafe => SAFE_DEFAULT_CHANNELS,
        };
        if let Err(e) = joystick.update(channels) {
            error!("Failed to update uinput: {}", e);
        }

        let now = Instant::now();
        failsafe_deadline =
            next_deadline(now, last_manual_time, last_autopilot_time, source_timeout);
    }

    session.close().await?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crsf_joystick::AXIS_MIN;

    #[test]
    fn manual_wins_when_fresh_and_sa_low() {
        let now = Instant::now();
        let timeout = Duration::from_millis(100);
        assert_eq!(
            select_source(Some(now), Some(now), AXIS_MIN, timeout),
            Source::Manual
        );
    }

    #[test]
    fn autopilot_wins_when_manual_stale() {
        let now = Instant::now();
        let timeout = Duration::from_millis(100);
        let manual = now - Duration::from_millis(200);
        let autopilot = now - Duration::from_millis(10);
        assert_eq!(
            select_source(Some(manual), Some(autopilot), AXIS_MIN, timeout),
            Source::Autopilot
        );
    }

    #[test]
    fn autopilot_wins_when_sa_high() {
        let now = Instant::now();
        let timeout = Duration::from_millis(100);
        assert_eq!(
            select_source(Some(now), Some(now), AXIS_MAX, timeout),
            Source::Autopilot
        );
    }

    #[test]
    fn failsafe_wins_when_autopilot_stale_and_sa_high() {
        let now = Instant::now();
        let timeout = Duration::from_millis(100);
        let autopilot = now - Duration::from_millis(200);
        assert_eq!(
            select_source(Some(now), Some(autopilot), AXIS_MAX, timeout),
            Source::Failsafe
        );
    }

    #[test]
    fn failsafe_wins_when_both_stale() {
        let now = Instant::now();
        let timeout = Duration::from_millis(100);
        let stale = now - Duration::from_millis(200);
        assert_eq!(
            select_source(Some(stale), Some(stale), AXIS_MIN, timeout),
            Source::Failsafe
        );
    }

    #[test]
    fn safe_default_channels_are_safe() {
        let c = SAFE_DEFAULT_CHANNELS;
        assert_eq!(c[0], AXIS_MID);
        assert_eq!(c[1], AXIS_MID);
        assert_eq!(c[2], AXIS_MIN);
        assert_eq!(c[3], AXIS_MID);
        assert_eq!(c[4], AXIS_MIN);
        assert_eq!(c[5], AXIS_MIN);
        assert_eq!(c[6], AXIS_MID);
        assert_eq!(c[7], AXIS_MIN);
        for &v in &c {
            assert!(v <= AXIS_MAX);
        }
    }

    #[test]
    fn next_deadline_is_earliest_future_staleness() {
        let now = Instant::now();
        let timeout = Duration::from_millis(100);
        let manual = now - Duration::from_millis(50);
        let autopilot = now - Duration::from_millis(20);
        let deadline = next_deadline(now, Some(manual), Some(autopilot), timeout);
        assert_eq!(deadline, manual + timeout);
    }

    #[test]
    fn next_deadline_ignores_already_stale_sources() {
        let now = Instant::now();
        let timeout = Duration::from_millis(100);
        let manual = now - Duration::from_millis(200);
        let autopilot = now - Duration::from_millis(20);
        let deadline = next_deadline(now, Some(manual), Some(autopilot), timeout);
        assert_eq!(deadline, autopilot + timeout);
    }
}
