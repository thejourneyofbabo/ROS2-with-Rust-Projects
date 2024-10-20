use futures::StreamExt;
use r2r::ad_msgs::msg::{LanePointDataArray, PolyfitLaneDataArray, VehicleInput, VehicleOutput};
use r2r::QosProfile;
use r2r::{geometry_msgs, std_msgs};
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "rust_multi_subscriber_node", "")?;
    let qos = QosProfile::default();
    println!("Node created: rust_multi_subscriber_node");

    let mut clicked_point_sub =
        node.subscribe::<geometry_msgs::msg::PointStamped>("/clicked_point", qos.clone())?;
    let mut lane_points_sub =
        node.subscribe::<LanePointDataArray>("/ego/lane_points_array", qos.clone())?;
    let mut poly_lanes_sub =
        node.subscribe::<PolyfitLaneDataArray>("/ego/poly_lanes", qos.clone())?;
    let mut vehicle_command_sub =
        node.subscribe::<VehicleInput>("/ego/vehicle_command", qos.clone())?;
    let mut vehicle_state_sub =
        node.subscribe::<VehicleOutput>("/ego/vehicle_state", qos.clone())?;
    let mut velocity_sub =
        node.subscribe::<std_msgs::msg::Float32>("/ego_vehicle_velocity", qos.clone())?;
    let mut roi_lanes_sub = node.subscribe::<LanePointDataArray>("/ego/ROI_lanes", qos.clone())?;

    println!("Subscribed to all topics. Waiting for messages...");

    loop {
        tokio::select! {
            msg = clicked_point_sub.next() => {
                if let Some(msg) = msg {
                    println!("Received /clicked_point: x={}, y={}, z={}", msg.point.x, msg.point.y, msg.point.z);
                }
            }
            msg = lane_points_sub.next() => {
                if let Some(msg) = msg {
                    println!("Received /ego/lane_points_array: frame_id={}, id={}", msg.frame_id, msg.id);
                }
            }
            msg = poly_lanes_sub.next() => {
                if let Some(msg) = msg {
                    println!("Received /ego/poly_lanes: frame_id={}", msg.frame_id);
                }
            }
            msg = vehicle_command_sub.next() => {
                if let Some(msg) = msg {
                    println!("Received /ego/vehicle_command: steering={}, accel={}, brake={}",
                             msg.steering, msg.accel, msg.brake);
                }
            }
            msg = vehicle_state_sub.next() => {
                if let Some(msg) = msg {
                    println!("Received /ego/vehicle_state: id={}, x={}, y={}, yaw={}, velocity={}, length={}, width={}",
                             msg.id, msg.x, msg.y, msg.yaw, msg.velocity, msg.length, msg.width);
                }
            }
            msg = velocity_sub.next() => {
                if let Some(msg) = msg {
                    println!("Received /ego_vehicle_velocity: {}", msg.data);
                }
            }
            msg = roi_lanes_sub.next() => {
                if let Some(msg) = msg {
                    println!("Received /ego/ROI_lanes: frame_id={}, id={}", msg.frame_id, msg.id);
                    for (i, lane) in msg.lane.iter().enumerate() {
                        println!("  Lane {}: frame_id={}, id={}", i, lane.frame_id, lane.id);
                        println!("    Points:");
                        for (j, point) in lane.point.iter().enumerate() {
                            println!("      Point {}: x={}, y={}, z={}", j, point.x, point.y, point.z);
                        }
                    }
                }
            }
            _ = tokio::time::sleep(Duration::from_millis(100)) => {
                node.spin_once(Duration::from_millis(0));
            }
        }
    }
}
