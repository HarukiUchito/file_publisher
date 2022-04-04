use std::process;
mod measurement;
use measurement::{read_file, to_ros_time};
use std::{thread::sleep, time::Duration};

use anyhow::{anyhow, Result};
use rclrust::{qos::QoSProfile, rclrust_info, ParameterValue};

const NODE_NAME: &str = "file_publisher_node";

fn main() -> Result<()> {
    // ros related initialization
    let ctx = rclrust::init()?;
    let node = ctx.create_node(NODE_NAME)?;

    let declare_and_get_param = |param, initial_value| {
        match node.declare_parameter(param, &initial_value) {
            Ok(_) => (),
            Err(e) => {
                println!("failed to declare ros parameter {}: {}", param, e);
                process::exit(-1);
            }
        };
        let p = match node.get_parameter(param) {
            Some(p) => p,
            None => {
                println!("ros parameter {} must be defined", param);
                process::exit(-1);
            }
        };
        println!("{:?} is set", p.value.get_value());
        return p.value.get_value().unwrap();
    };
    let base_link_frame_name =
        declare_and_get_param("base_link_frame_name", ParameterValue::string("base_link"))
            .to_string()
            .unwrap();
    let _play_speed = declare_and_get_param("play_speed", ParameterValue::double(0.1));

    let pub_imu =
        node.create_publisher::<rclrust_msg::sensor_msgs::msg::Imu>("imu", &QoSProfile::default())?;
    let pub_twist = node
        .create_publisher::<rclrust_msg::geometry_msgs::msg::TwistWithCovarianceStamped>(
            "twist",
            &QoSProfile::default(),
        )?;

    println!("reading file..");
    let mut measurements = match read_file("/home/xoke/rosData/meijo/run1/imu-cp.csv") {
        Ok(result) => result,
        Err(err) => {
            println!("error running example: {}", err);
            process::exit(1);
        }
    };

    loop {
        let mstart_time = measurements[0].time;
        let start_time = rclrust::Clock::ros().unwrap().now().unwrap();
        let mut cnt = 0;
        let mut iter = measurements.iter_mut();
        while let Some(m) = iter.next() {
            let md = m.time - mstart_time;
            cnt += 1;
            let mut imu_msg = m.imu.clone();
            imu_msg.header.frame_id = base_link_frame_name.clone();
            let mut twist_msg = m.twist.clone();
            twist_msg.header.frame_id = base_link_frame_name.clone();
            loop {
                let ct = rclrust::Clock::ros().unwrap().now().unwrap().nanosecs;
                let td = (ct - start_time.nanosecs) as f64 * 1e-9;
                if td >= md {
                    let rt = ct as f64 * 1e-9;
                    imu_msg.header.stamp = to_ros_time(rt);
                    pub_imu.publish(&imu_msg)?;
                    twist_msg.header.stamp = to_ros_time(rt);
                    pub_twist.publish(&twist_msg)?;
                    println!(
                        "published imu {} on file time: {}, unix time: {}",
                        cnt,
                        m.time,
                        ct as f64 * 1e-9
                    );
                    break;
                }
                sleep(Duration::from_millis(10));
            }
        }
    }
}
