mod physics;

use anyhow::Result;
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions};
use ros_env::drone_interfaces::msg::{AimSolution, TargetObservation};

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("water_gun_zeroing")?;

    // Publish an explicit aiming solution instead of an anonymous array.
    let publisher = node.create_publisher::<AimSolution>("water_gun/aim_angles")?;

    // Receive a target observation with named fields and units.
    let _subscription =
        node.create_subscription("lidar/target_info", move |msg: TargetObservation| {
            let distance_m = msg.distance_m;
            let horizontal_angle_deg = msg.horizontal_angle_deg;
            let vertical_angle_deg = msg.vertical_angle_deg;

            match physics::calculate_aiming_angles(
                distance_m,
                horizontal_angle_deg,
                vertical_angle_deg,
            ) {
                Some((yaw_deg, pitch_deg)) => {
                    println!("Target solution -> yaw: {yaw_deg:.2}°, pitch: {pitch_deg:.2}°");

                    let mut output = AimSolution::default();
                    output.yaw_deg = yaw_deg;
                    output.pitch_deg = pitch_deg;

                    if let Err(error) = publisher.publish(&output) {
                        eprintln!("Unable to publish aiming solution: {error}");
                    }
                }
                None => eprintln!("Target is invalid or out of range"),
            }
        })?;

    println!("Water gun zeroing node started");
    executor.spin(SpinOptions::default()).first_error()?;

    Ok(())
}
