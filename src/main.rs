use std::env;
use std::io::{self, Read};
use std::time::Duration;

use ldlidar::{BAUD_RATE, FrameDecoder};
use serialport::{DataBits, FlowControl, Parity, StopBits};

const DEFAULT_PORT: &str = "/dev/ttyUSB0";
const READ_TIMEOUT_MS: u64 = 100;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port_name = env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_PORT.to_string());

    let mut port = serialport::new(&port_name, BAUD_RATE)
        .data_bits(DataBits::Eight)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .flow_control(FlowControl::None)
        .timeout(Duration::from_millis(READ_TIMEOUT_MS))
        .open()
        .map_err(|error| format!("failed to open serial port {port_name}: {error}"))?;

    eprintln!("reading STL-27L frames from {port_name} at {BAUD_RATE} 8N1");

    let mut decoder = FrameDecoder::new();
    let mut buffer = [0_u8; 512];

    loop {
        match port.read(&mut buffer) {
            Ok(bytes_read) if bytes_read > 0 => {
                for frame in decoder.push_bytes(&buffer[..bytes_read]) {
                    match frame {
                        Ok(frame) => {
                            println!(
                                "speed={}deg/s start={:.2}deg end={:.2}deg timestamp={}ms points={}",
                                frame.speed_degrees_per_second,
                                frame.start_angle_degrees,
                                frame.end_angle_degrees,
                                frame.timestamp_ms,
                                frame.points.len()
                            );

                            for point in frame.points {
                                println!(
                                    "  angle={:.2}deg distance={}mm intensity={}",
                                    point.angle_degrees, point.distance_mm, point.intensity
                                );
                            }
                        }
                        Err(error) => eprintln!("dropped invalid frame: {error:?}"),
                    }
                }
            }
            Ok(_) => {}
            Err(error) if error.kind() == io::ErrorKind::TimedOut => {}
            Err(error) => return Err(Box::new(error)),
        }
    }
}
