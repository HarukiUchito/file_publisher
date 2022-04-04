use anyhow::{anyhow, Context, Result};
use rclrust_msg::geometry_msgs;

use serde::Deserialize;

use rclrust_msg::sensor_msgs::msg::Imu;

pub fn to_ros_time(time: f64) -> rclrust_msg::builtin_interfaces::msg::Time {
    let sec = f64::floor(time);
    let nsec: u32 = ((time - sec) * 1e9) as u32;
    rclrust_msg::builtin_interfaces::msg::Time {
        sec: sec as i32,
        nanosec: nsec as u32,
    }
}

#[derive(Debug, Deserialize)]
struct Record {
    gps_tow: Option<f64>,
    _gps_week: Option<i32>,
    acc_x: Option<f64>,
    acc_y: Option<f64>,
    acc_z: Option<f64>,
    ang_vel_x: Option<f64>,
    ang_vel_y: Option<f64>,
    ang_vel_z: Option<f64>,
    wheel_vel: Option<f64>,
}

pub struct Measurement {
    pub time: f64,
    pub imu: Imu,
    pub twist: geometry_msgs::msg::TwistWithCovarianceStamped,
}

pub fn read_file(path: &str) -> Result<Vec<Measurement>> {
    let mut rdr = csv::Reader::from_path(path)?;
    let mut cnt = 0;
    let mut measurements: Vec<Measurement> = Vec::new();

    let mut start_time = -1.0;
    for result in rdr.deserialize() {
        let record: Record = result?;

        let ftime = record.gps_tow.context("invalid gps_tow")?;
        if start_time <= 0.0 {
            start_time = ftime;
        }
        let rostime = to_ros_time(ftime);
        let acc_x = record.ang_vel_x.context("invalid ang_vel_x value")?;
        let acc_y = record.ang_vel_y.context("invalid ang_vel_y value")?;
        let acc_z = record.ang_vel_z.context("invalid ang_vel_z value")?;

        let imu_msg = Imu {
            header: rclrust_msg::std_msgs::msg::Header {
                stamp: rostime.clone(),
                frame_id: "base_link".to_string(),
            },
            linear_acceleration: geometry_msgs::msg::Vector3 {
                x: record.acc_x.context("invalid acc_x value")?,
                y: record.acc_y.context("invalid acc_y value")?,
                z: record.acc_z.context("invalid acc_z value")?,
            },
            linear_acceleration_covariance: [0.0; 9],
            angular_velocity: geometry_msgs::msg::Vector3 {
                x: acc_x,
                y: acc_y,
                z: acc_z,
            },
            angular_velocity_covariance: [0.0; 9],
            orientation: geometry_msgs::msg::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
            },
            orientation_covariance: [0.0; 9],
        };

        let twist_msg = geometry_msgs::msg::TwistWithCovarianceStamped {
            header: rclrust_msg::std_msgs::msg::Header {
                stamp: rostime,
                frame_id: "base_link".to_string(),
            },
            twist: geometry_msgs::msg::TwistWithCovariance {
                twist: geometry_msgs::msg::Twist {
                    linear: geometry_msgs::msg::Vector3 {
                        x: record.wheel_vel.context("invalid wheel_vel value")?,
                        y: 0.0,
                        z: 0.0,
                    },
                    angular: geometry_msgs::msg::Vector3 {
                        x: acc_x,
                        y: acc_y,
                        z: acc_z,
                    },
                },
                covariance: [0.0; 36],
            },
        };

        measurements.push(Measurement {
            time: ftime - start_time,
            imu: imu_msg,
            twist: twist_msg,
        });

        cnt += 1;
        if cnt > 2 {
            //break;
        }
    }
    Ok(measurements)
}
