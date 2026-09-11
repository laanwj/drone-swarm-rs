use chrono::{DateTime, Utc};
use clap::Parser;
use telemetry_lib::crsf::{self, CrsfPacket};
use telemetry_lib::topics;
use log::{debug, info, warn};
use metrics::{Unit, counter, describe_counter};
use metrics_exporter_statsd::StatsdBuilder;
use serde_json::Value;
use std::sync::Arc;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::TcpListener;
use tokio::time::{Duration, interval};
use zenoh::Config;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Bind address for GPSD service.
    #[arg(long, default_value = "127.0.0.1:2947")]
    gpsd_bind: std::net::SocketAddr,

    /// GPS position update frequency.
    #[arg(short, long, default_value_t = 10)]
    frequency: u64,

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

// NMEA formatting helpers
fn format_nmea(body: &str) -> String {
    let mut checksum = 0u8;
    for b in body.bytes() {
        checksum ^= b;
    }
    format!("${}*{:02X}\r\n", body, checksum)
}

fn to_nmea_coord(val: f64, is_lat: bool) -> (String, char) {
    let abs_val = val.abs();
    let deg = abs_val.floor();
    let min = (abs_val - deg) * 60.0;

    let format_str = if is_lat {
        format!("{:02}{:07.4}", deg as u32, min)
    } else {
        format!("{:03}{:07.4}", deg as u32, min)
    };

    let dir = if is_lat {
        if val >= 0.0 { 'N' } else { 'S' }
    } else {
        if val >= 0.0 { 'E' } else { 'W' }
    };

    (format_str, dir)
}

fn generate_gga(time: DateTime<Utc>, lat: f64, lon: f64, alt: f64, sats: u32) -> String {
    let (lat_str, lat_dir) = to_nmea_coord(lat, true);
    let (lon_str, lon_dir) = to_nmea_coord(lon, false);
    let time_str = time.format("%H%M%S.%3f");

    // $GPGGA,hhmmss.ss,llll.ll,a,yyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
    let body = format!(
        "GPGGA,{},{},{},{},{},1,{:02},0.9,{:.1},M,46.9,M,,",
        time_str, lat_str, lat_dir, lon_str, lon_dir, sats, alt
    );
    format_nmea(&body)
}

fn generate_gga_nofix(time: DateTime<Utc>) -> String {
    let time_str = time.format("%H%M%S.%3f");

    // $GPGGA,hhmmss.ss,,,,,0,00,99.99,,,,,,*hh
    let body = format!("GPGGA,{},,,,,0,00,99.99,,,,,,", time_str);
    format_nmea(&body)
}

fn generate_rmc(time: DateTime<Utc>, lat: f64, lon: f64, speed_knots: f64, course: f64) -> String {
    let (lat_str, lat_dir) = to_nmea_coord(lat, true);
    let (lon_str, lon_dir) = to_nmea_coord(lon, false);
    let time_str = time.format("%H%M%S.%3f");
    let date_str = time.format("%d%m%y");

    // $GPRMC,hhmmss.ss,A,llll.ll,a,yyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
    let body = format!(
        "GPRMC,{},A,{},{},{},{},{:.1},{:.1},{},,,A",
        time_str, lat_str, lat_dir, lon_str, lon_dir, speed_knots, course, date_str
    );
    format_nmea(&body)
}

fn generate_rmc_nofix(time: DateTime<Utc>) -> String {
    let time_str = time.format("%H%M%S.%3f");
    let date_str = time.format("%d%m%y");

    // $GPRMC,hhmmss.ss,V,,,,,,,ddmmyy,,*hh
    let body = format!("GPRMC,{},V,,,,,,,{},,", time_str, date_str);
    format_nmea(&body)
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    env_logger::init();
    let args = Args::parse();

    info!("Starting crsf-gpsd on {}", args.gpsd_bind);

    if args.metrics_statsd {
        let host = args.metrics_statsd_destination.ip().to_string();
        let recorder = StatsdBuilder::from(&host, args.metrics_statsd_destination.port())
            .build(None)
            .expect("failed to create StatsD recorder");
        metrics::set_global_recorder(recorder)
            .expect("failed to install StatsD recorder");
    }

    describe_counter!(
        "gpsd.telemetry.rx",
        Unit::Count,
        "Telemetry packets received"
    );
    describe_counter!("gpsd.client.accept", Unit::Count, "Clients accepted");
    describe_counter!("gpsd.nmea.tx", Unit::Count, "NMEA sentences sent");

    // Zenoh session
    let mut config = Config::default();
    config.insert_json5("mode", &format!(r#""{}""#, args.zenoh_mode))?;
    if let Some(ref endpoint) = args.zenoh_connect {
        config.insert_json5("connect/endpoints", &format!(r#"["{}"]"#, endpoint))?;
    }

    let session = zenoh::open(config).await?;
    let crsf_tel_topic = topics::topic(&args.zenoh_prefix, topics::CRSF_TELEMETRY);
    info!("Subscribing to: {}", crsf_tel_topic);
    let crsf_tel_subscriber = session.declare_subscriber(&crsf_tel_topic).await?;

    // Shared state for latest GPS from CRSF telemetry
    let shared_state: Arc<std::sync::RwLock<Option<(std::time::Instant, crsf::Gps)>>> =
        Arc::new(std::sync::RwLock::new(None));
    let tx = shared_state.clone();
    let rx = shared_state.clone();

    // CRSF telemetry reader task — extract GPS packets
    tokio::spawn(async move {
        loop {
            match crsf_tel_subscriber.recv_async().await {
                Ok(sample) => {
                    let payload = sample.payload().to_bytes();
                    counter!("gpsd.telemetry.rx").increment(1);
                    if let Some(CrsfPacket::Gps(gps)) = crsf::parse_packet_check(&payload) {
                        if let Ok(mut lock) = tx.write() {
                            *lock = Some((std::time::Instant::now(), gps));
                        }
                    }
                }
                Err(e) => {
                    warn!("CRSF telemetry subscriber error: {}", e);
                    break;
                }
            }
        }
    });

    // TCP Listener for GPSD clients
    let listener = TcpListener::bind(&args.gpsd_bind).await?;

    loop {
        let (mut socket, addr) = listener.accept().await?;
        info!("Accepted connection from {}", addr);
        counter!("gpsd.client.accept").increment(1);
        let rx = rx.clone();
        let freq = args.frequency;

        tokio::spawn(async move {
            let (reader, mut writer) = socket.split();
            let mut reader = BufReader::new(reader);

            // Send banner
            let banner = r#"{"class":"VERSION","release":"2.93","rev":"2010-03-30T12:18:17", "proto_major":3,"proto_minor":2}"#;
            writer
                .write_all(format!("{}\n", banner).as_bytes())
                .await
                .ok();

            // Read ?WATCH command (terminated by ;)
            let mut line_bytes = Vec::new();
            if let Ok(_) = reader.read_until(b';', &mut line_bytes).await {
                let line_raw = String::from_utf8_lossy(&line_bytes);
                let line = line_raw.trim();

                if line.starts_with("?WATCH=") && line.ends_with(';') {
                    let json_str = &line[7..line.len() - 1]; // Strip ?WATCH= and ;

                    let valid = if let Ok(val) = serde_json::from_str::<Value>(json_str) {
                        let enable = val.get("enable").and_then(|v| v.as_bool()).unwrap_or(false);
                        let nmea = val.get("nmea").and_then(|v| v.as_bool()).unwrap_or(false);
                        let raw = val.get("raw").and_then(|v| v.as_bool()).unwrap_or(false);

                        enable && nmea && raw
                    } else {
                        false
                    };

                    if valid {
                        let mut interval = interval(Duration::from_millis(1000 / freq));
                        loop {
                            interval.tick().await;

                            let packet_data = if let Ok(lock) = rx.read() {
                                lock.clone()
                            } else {
                                None
                            };

                            let time = Utc::now();
                            let mut sentences = Vec::<String>::new();
                            let mut have_fix = false;
                            if let Some((recv_time, ref gps)) = packet_data {
                                if recv_time.elapsed() < Duration::from_secs(10) {
                                    debug!("in {:?}", gps);
                                    let lat = gps.lat_deg();
                                    let lon = gps.lon_deg();
                                    let alt = gps.alt_m();
                                    let knots = gps.speed_kmh() / 1.852;
                                    let course = gps.heading_deg();

                                    sentences.push(generate_gga(time, lat, lon, alt, gps.sats as u32));
                                    sentences.push(generate_rmc(time, lat, lon, knots, course));
                                    have_fix = true;
                                }
                            }

                            if !have_fix {
                                // Send invalid GPS fix
                                sentences.push(generate_gga_nofix(time));
                                sentences.push(generate_rmc_nofix(time));
                            }

                            for sentence_out in sentences {
                                debug!("out {}", sentence_out);
                                writer.write_all(sentence_out.as_bytes()).await.ok();
                                counter!("gpsd.nmea.tx").increment(1);
                            }
                        }
                    } else {
                        warn!("Invalid WATCH command: {}", line);
                    }
                }
            }
        });
    }
}
