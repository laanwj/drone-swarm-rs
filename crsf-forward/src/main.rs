use clap::Parser;
use telemetry_lib::crsf::{self};
use telemetry_lib::topics;
use log::{error, info, trace, warn};
use metrics::{Unit, counter, describe_counter, describe_histogram, histogram};
use metrics_exporter_statsd::StatsdBuilder;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio_serial::SerialPortBuilderExt;
use zenoh::Config;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Serial port to use.
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    port: String,

    /// Serial baudrate to use.
    #[arg(short, long, default_value_t = 420000)]
    baud: u32,

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

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    env_logger::init();
    let args = Args::parse();

    if args.metrics_statsd {
        let host = args.metrics_statsd_destination.ip().to_string();
        let recorder = StatsdBuilder::from(&host, args.metrics_statsd_destination.port())
            .build(None)
            .expect("failed to create StatsD recorder");
        metrics::set_global_recorder(recorder)
            .expect("failed to install StatsD recorder");
    }

    describe_counter!(
        "crsf.tx.count",
        Unit::Count,
        "Sent telemetry CRSF packet count"
    );
    describe_counter!("crsf.rx.count", Unit::Count, "Received CRSF packet count");
    describe_counter!(
        "crsf.rx.valid",
        Unit::Count,
        "Valid received CRSF packet count"
    );
    describe_counter!(
        "crsf.tx.crc_err",
        Unit::Count,
        "Number of almost-sent CRSF packets with CRC mismatch"
    );
    describe_counter!(
        "crsf.rx.crc_err",
        Unit::Count,
        "Number of received CRSF packets with CRC mismatch"
    );
    describe_histogram!("crsf.rx.frame_size", Unit::Bytes, "Receive frame size");
    describe_histogram!(
        "crsf.tx.frame_size",
        Unit::Bytes,
        "Sent telemetry frame size"
    );

    info!("Starting crsf-forward");
    info!("Serial Port: {} @ {}", args.port, args.baud);

    let port = tokio_serial::new(&args.port, args.baud).open_native_async()?;

    // Zenoh session
    let mut config = Config::default();
    config.insert_json5("mode", &format!(r#""{}""#, args.zenoh_mode))?;
    if let Some(ref endpoint) = args.zenoh_connect {
        config.insert_json5("connect/endpoints", &format!(r#"["{}"]"#, endpoint))?;
    }

    let session = zenoh::open(config).await?;

    let crsf_tel_topic = topics::topic(&args.zenoh_prefix, topics::CRSF_TELEMETRY);
    let crsf_rc_topic = topics::topic(&args.zenoh_prefix, topics::CRSF_RC);

    info!("Subscribing to: {}", crsf_tel_topic);
    info!("Publishing on: {}", crsf_rc_topic);

    let tel_subscriber = session.declare_subscriber(&crsf_tel_topic).await?;
    let rc_publisher = session.declare_publisher(crsf_rc_topic).await?;

    let (mut reader, mut writer) = tokio::io::split(port);

    // Task: Zenoh CRSF telemetry -> Serial (with CRC check)
    let mut writer_handle = tokio::spawn(async move {
        loop {
            match tel_subscriber.recv_async().await {
                Ok(sample) => {
                    let frame = sample.payload().to_bytes();
                    let frame_size = frame.len();
                    if frame_size > crsf::MAX_FRAME_SIZE {
                        warn!("Packet too large: {}", frame_size);
                        continue;
                    }

                    trace!("tx: {:02x?}", &*frame);
                    counter!("crsf.tx.count").increment(1);
                    histogram!("crsf.tx.frame_size").record(frame.len() as f64);

                    if !crsf::frame_check_crc(&frame) {
                        trace!("Invalid CRC on incoming telemetry packet");
                        counter!("crsf.tx.crc_err").increment(1);
                        continue;
                    }

                    if let Err(e) = writer.write_all(&frame).await {
                        error!("Serial write error: {}", e);
                        break;
                    }
                }
                Err(e) => {
                    error!("Telemetry subscriber error: {}", e);
                    break;
                }
            }
        }
    });

    // Task: Serial -> Zenoh (RC channels)
    let mut reader_handle = tokio::spawn(async move {
        let mut buf = Vec::new(); // Buffer for incoming data
        let mut tmp = [0u8; 1024];

        loop {
            match reader.read(&mut tmp).await {
                Ok(0) => {
                    // EOF
                    error!("Serial EOF");
                    break;
                }
                Ok(n) => {
                    buf.extend_from_slice(&tmp[0..n]);

                    // Process buffer
                    loop {
                        // Find sync byte (we are the flight controller, in this context).
                        if let Some(pos) = buf
                            .iter()
                            .position(|&b| b == crsf::device_address::FLIGHT_CONTROLLER)
                        {
                            // Trim garbage before sync
                            if pos > 0 {
                                buf.drain(0..pos);
                            }

                            // Check length
                            if buf.len() < 2 {
                                break; // Need more data
                            }
                            let len = buf[1] as usize; // Length of Payload + CRC
                            let total_len = len + 2; // Sync + Len + Payload + CRC

                            if total_len > crsf::MAX_FRAME_SIZE {
                                // "Each CRSF frame is not longer than 64 bytes (including the Sync and CRC bytes)"
                                // This packet would be too long. Drop sync byte and try again.
                                buf.remove(0);
                                continue;
                            }
                            if buf.len() < total_len {
                                break; // Need more data
                            }
                            counter!("crsf.rx.count").increment(1);
                            histogram!("crsf.rx.frame_size").record(total_len as f64);

                            // Full packet found
                            let frame = &buf[0..total_len];
                            // Verify CRC
                            let payload = &frame[2..total_len - 1];
                            let crc_byte = frame[total_len - 1];

                            if crsf::calc_crc8(payload) == crc_byte {
                                // Valid packet
                                trace!("rx: {:02x?}", payload);
                                counter!("crsf.rx.valid").increment(1);
                                if let Err(e) = rc_publisher.put(frame).await {
                                    warn!("Zenoh publish error: {}", e);
                                }
                            } else {
                                trace!("CRC mismatch");
                                counter!("crsf.rx.crc_err").increment(1);
                            }

                            buf.drain(0..total_len);
                        } else {
                            // No sync found, clear buffer
                            buf.clear();
                            break;
                        }
                    }
                }
                Err(e) => {
                    error!("Serial read error: {}", e);
                    break;
                }
            }
        }
    });

    tokio::select! {
        _ = &mut writer_handle => error!("Writer task finished"),
        _ = &mut reader_handle => error!("Reader task finished"),
    }

    session.close().await?;
    Ok(())
}
