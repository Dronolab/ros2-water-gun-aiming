use std::f64::consts::PI;

// Constants (to be calibrated later)
pub const WATER_VELOCITY: f64 = 15.0; // m/s
pub const GRAVITY: f64 = 9.81; // m/s^2

// Offsets from LiDAR to water gun (m)
pub const OFFSET_X: f64 = 0.1; // Forward
pub const OFFSET_Y: f64 = 0.0; // Left
pub const OFFSET_Z: f64 = -0.1; // Down

/// Calculate the required yaw and pitch angles for the water gun to hit the target.
/// Returns Option<(yaw_deg, pitch_deg)>. None if the target is out of range.
pub fn calculate_aiming_angles(
    lidar_distance_m: f64,
    lidar_horizontal_angle_deg: f64, // Yaw (0 is forward, positive is left)
    lidar_vertical_angle_deg: f64,   // Pitch (0 is forward, positive is up)
) -> Option<(f64, f64)> {
    if lidar_distance_m <= 0.0
        || !lidar_distance_m.is_finite()
        || !lidar_horizontal_angle_deg.is_finite()
        || !lidar_vertical_angle_deg.is_finite()
    {
        return None;
    }

    let h_rad = lidar_horizontal_angle_deg * PI / 180.0;
    let v_rad = lidar_vertical_angle_deg * PI / 180.0;

    // 1. Convert LiDAR spherical to Cartesian (relative to LiDAR)
    // Assuming standard ROS frame: X forward, Y left, Z up
    let lidar_x = lidar_distance_m * v_rad.cos() * h_rad.cos();
    let lidar_y = lidar_distance_m * v_rad.cos() * h_rad.sin();
    let lidar_z = lidar_distance_m * v_rad.sin();

    // 2. Apply offsets to get target coordinates relative to water gun
    let target_x = lidar_x - OFFSET_X;
    let target_y = lidar_y - OFFSET_Y;
    let target_z = lidar_z - OFFSET_Z;

    // 3. Calculate yaw
    let target_yaw_rad = target_y.atan2(target_x);
    let target_yaw_deg = target_yaw_rad * 180.0 / PI;

    // 4. Calculate pitch
    // Horizontal distance to target from the gun
    let target_dist_xy = (target_x * target_x + target_y * target_y).sqrt();

    // Use analytical formula for projectile motion without drag
    let v = WATER_VELOCITY;
    let g = GRAVITY;
    let x = target_dist_xy;
    let y = target_z;

    let v2 = v * v;
    let v4 = v2 * v2;

    // The discriminant of the quadratic equation
    let discriminant = v4 - g * (g * x * x + 2.0 * y * v2);

    // If discriminant is less than 0, the target is out of range
    if discriminant < 0.0 {
        return None;
    }

    // We use the minus sign for the lower, direct-fire trajectory
    let root = discriminant.sqrt();
    let target_pitch_rad = (v2 - root).atan2(g * x);
    let target_pitch_deg = target_pitch_rad * 180.0 / PI;

    Some((target_yaw_deg, target_pitch_deg))
}

#[cfg(test)]
mod tests {
    use super::calculate_aiming_angles;

    #[test]
    fn computes_a_direct_shot_in_front_of_the_drone() {
        let (yaw_deg, pitch_deg) = calculate_aiming_angles(10.0, 0.0, 0.0).unwrap();

        assert!(yaw_deg.abs() < 0.01);
        assert!(pitch_deg > 0.0);
        assert!(pitch_deg < 45.0);
    }

    #[test]
    fn keeps_a_lateral_target_near_ninety_degrees() {
        let (yaw_deg, _) = calculate_aiming_angles(10.0, 90.0, 0.0).unwrap();

        assert!((80.0..100.0).contains(&yaw_deg));
    }

    #[test]
    fn rejects_invalid_measurements() {
        assert!(calculate_aiming_angles(0.0, 0.0, 0.0).is_none());
        assert!(calculate_aiming_angles(-1.0, 0.0, 0.0).is_none());
        assert!(calculate_aiming_angles(f64::NAN, 0.0, 0.0).is_none());
    }

    #[test]
    fn rejects_a_target_beyond_the_ballistic_range() {
        assert!(calculate_aiming_angles(1_000.0, 0.0, 0.0).is_none());
    }
}
