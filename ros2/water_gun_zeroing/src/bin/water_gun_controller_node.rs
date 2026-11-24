use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions};
use std::{
    error::Error,
    io::Write,
    sync::{Arc, Mutex},
    time::Duration,
};

use ros_env::drone_interfaces::{
    msg::AimSolution,
    srv::{Shoot, Shoot_Request, Shoot_Response},
};

pub const WATER_GUN_PORT: &str = "/dev/ttyACM0";
pub const WATER_GUN_BAUDRATE: u32 = 115_200;

pub const MIN_PITCH_DEG: f64 = -90.0;
pub const MAX_PITCH_DEG: f64 = 25.0;
pub const MIN_YAW_DEG: f64 = -90.0;
pub const MAX_YAW_DEG: f64 = 90.0;

pub const MIN_SHOOT_DURATION_MS: i32 = 500;
pub const MAX_SHOOT_DURATION_MS: i32 = 2_000;

pub fn aiming_command(aim_solution: &AimSolution) -> Result<String, String> {
    if !aim_solution.pitch_deg.is_finite() || !aim_solution.yaw_deg.is_finite() {
        return Err("Aiming angles must be finite numbers".to_string());
    }

    if !(MIN_PITCH_DEG..=MAX_PITCH_DEG).contains(&aim_solution.pitch_deg) {
        return Err(format!(
            "Pitch must be between {MIN_PITCH_DEG} and {MAX_PITCH_DEG} degrees"
        ));
    }

    if !(MIN_YAW_DEG..=MAX_YAW_DEG).contains(&aim_solution.yaw_deg) {
        return Err(format!(
            "Yaw must be between {MIN_YAW_DEG} and {MAX_YAW_DEG} degrees"
        ));
    }

    Ok(format!(
        "<pitch{};yaw{}>\n",
        aim_solution.pitch_deg.round() as i32,
        aim_solution.yaw_deg.round() as i32
    ))
}

pub fn shooting_command(duration_ms: i32) -> Result<String, String> {
    if !(MIN_SHOOT_DURATION_MS..=MAX_SHOOT_DURATION_MS).contains(&duration_ms) {
        return Err(format!(
            "Shoot duration must be between {MIN_SHOOT_DURATION_MS} and {MAX_SHOOT_DURATION_MS} ms"
        ));
    }

    Ok(format!("<shoot{duration_ms}>\n"))
}

fn main() -> Result<(), Box<dyn Error>> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    let node = executor.create_node("water_gun_controller")?;

    let serial_port = Arc::new(Mutex::new(
        serialport::new(WATER_GUN_PORT, WATER_GUN_BAUDRATE)
            .timeout(Duration::from_millis(100))
            .open()?,
    ));

    println!("Water-gun controller node started");

    let serial_port_for_aiming = Arc::clone(&serial_port);

    let _subscription =
        node.create_subscription("water_gun/aim_angles", move |aim_solution: AimSolution| {
            let command = match aiming_command(&aim_solution) {
                Ok(command) => command,
                Err(message) => {
                    eprintln!("Rejected aiming command: {message}");
                    return;
                }
            };

            println!(
                "Aiming command: yaw={:.2}°, pitch={:.2}° -> {}",
                aim_solution.yaw_deg,
                aim_solution.pitch_deg,
                command.trim()
            );

            let mut serial_port = match serial_port_for_aiming.lock() {
                Ok(port) => port,
                Err(error) => {
                    eprintln!(
                        "Unable to access serial port: {error}. \
                        Restart water_gun_controller."
                    );
                    std::process::exit(1);
                }
            };

            if let Err(error) = serial_port.write_all(command.as_bytes()) {
                eprintln!(
                    "Serial connection lost while sending aiming command: {error}. \
                        Restart water_gun_controller."
                );
                std::process::exit(1);
            }
        })?;

    let serial_port_for_shooting = Arc::clone(&serial_port);

    let _shoot_service =
        node.create_service::<Shoot, _>("water_gun/shoot", move |request: Shoot_Request| {
            let command = match shooting_command(request.duration_ms) {
                Ok(command) => command,
                Err(message) => {
                    eprintln!("Rejected shooting command: {message}");

                    return Shoot_Response {
                        sent: false,
                        message,
                    };
                }
            };

            let mut serial_port = match serial_port_for_shooting.lock() {
                Ok(port) => port,
                Err(error) => {
                    eprintln!(
                        "Unable to access serial port: {error}. \
                        Restart water_gun_controller."
                    );
                    std::process::exit(1);
                }
            };

            match serial_port.write_all(command.as_bytes()) {
                Ok(()) => {
                    println!("Shooting command sent: {}", command.trim());

                    Shoot_Response {
                        sent: true,
                        message: format!("Shooting command sent for {} ms", request.duration_ms),
                    }
                }
                Err(error) => {
                    eprintln!(
                        "Serial connection lost while sending shooting command: {error}. \
                        Restart water_gun_controller."
                    );
                    std::process::exit(1);
                }
            }
        })?;

    executor.spin(SpinOptions::default()).first_error()?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn aim_solution(pitch_deg: f64, yaw_deg: f64) -> AimSolution {
        let mut solution = AimSolution::default();
        solution.pitch_deg = pitch_deg;
        solution.yaw_deg = yaw_deg;
        solution
    }

    #[test]
    fn formats_valid_aiming_command() {
        let solution = aim_solution(13.4, -7.6);

        assert_eq!(
            aiming_command(&solution),
            Ok("<pitch13;yaw-8>\n".to_string())
        );
    }

    #[test]
    fn accepts_aiming_boundaries() {
        assert!(aiming_command(&aim_solution(MIN_PITCH_DEG, MIN_YAW_DEG)).is_ok());
        assert!(aiming_command(&aim_solution(MAX_PITCH_DEG, MAX_YAW_DEG)).is_ok());
    }

    #[test]
    fn rejects_pitch_outside_limits() {
        assert!(aiming_command(&aim_solution(MIN_PITCH_DEG - 0.1, 0.0)).is_err());
        assert!(aiming_command(&aim_solution(MAX_PITCH_DEG + 0.1, 0.0)).is_err());
    }

    #[test]
    fn rejects_yaw_outside_limits() {
        assert!(aiming_command(&aim_solution(0.0, MIN_YAW_DEG - 0.1)).is_err());
        assert!(aiming_command(&aim_solution(0.0, MAX_YAW_DEG + 0.1)).is_err());
    }

    #[test]
    fn rejects_non_finite_aiming_angles() {
        assert!(aiming_command(&aim_solution(f64::NAN, 0.0)).is_err());
        assert!(aiming_command(&aim_solution(0.0, f64::INFINITY)).is_err());
    }

    #[test]
    fn formats_valid_shooting_command() {
        assert_eq!(shooting_command(750), Ok("<shoot750>\n".to_string()));
    }

    #[test]
    fn accepts_shooting_duration_boundaries() {
        assert!(shooting_command(MIN_SHOOT_DURATION_MS).is_ok());
        assert!(shooting_command(MAX_SHOOT_DURATION_MS).is_ok());
    }

    #[test]
    fn rejects_shooting_duration_below_minimum() {
        assert!(shooting_command(MIN_SHOOT_DURATION_MS - 1).is_err());
    }

    #[test]
    fn rejects_shooting_duration_above_maximum() {
        assert!(shooting_command(MAX_SHOOT_DURATION_MS + 1).is_err());
    }
}
