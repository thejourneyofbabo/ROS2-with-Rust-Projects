use futures::StreamExt;
use r2r::ad_msgs::msg::{LanePointDataArray, PolyfitLaneDataArray, VehicleInput, VehicleOutput};
use r2r::QosProfile;
use r2r::{geometry_msgs, std_msgs};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::time::interval;

#[derive(Debug, Clone)]
struct LaneStats {
    avg_x: f64,
    avg_y: f64,
    avg_z: f64,
    last_update: Instant,
}

impl LaneStats {
    fn new() -> Self {
        Self {
            avg_x: 0.0,
            avg_y: 0.0,
            avg_z: 0.0,
            last_update: Instant::now(),
        }
    }

    fn update(&mut self, points: &[geometry_msgs::msg::Point]) {
        if points.is_empty() {
            return;
        }
        let sum = points.iter().fold((0.0, 0.0, 0.0), |acc, p| {
            (acc.0 + p.x, acc.1 + p.y, acc.2 + p.z)
        });
        let count = points.len() as f64;
        self.avg_x = sum.0 / count;
        self.avg_y = sum.1 / count;
        self.avg_z = sum.2 / count;
        self.last_update = Instant::now();
    }
}

#[derive(Debug, Clone)]
struct VehicleState {
    id: String,
    x: f64,
    y: f64,
    yaw: f64,
    velocity: f64,
    last_update: Instant,
}

impl VehicleState {
    fn new() -> Self {
        Self {
            id: String::new(),
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
            velocity: 0.0,
            last_update: Instant::now(),
        }
    }

    fn update(&mut self, msg: &VehicleOutput) {
        self.id = msg.id.clone();
        self.x = msg.x;
        self.y = msg.y;
        self.yaw = msg.yaw;
        self.velocity = msg.velocity;
        self.last_update = Instant::now();
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "rust_multi_subscriber_node", "")?;
    let qos = QosProfile::default();
    println!("Node created: rust_multi_subscriber_node");

    let lane_stats = Arc::new(Mutex::new(vec![LaneStats::new(), LaneStats::new()]));
    let vehicle_state = Arc::new(Mutex::new(VehicleState::new()));

    // ROI Lanes 구독
    let lane_stats_clone = lane_stats.clone();
    let mut roi_lanes_sub = node.subscribe::<LanePointDataArray>("/ego/ROI_lanes", qos.clone())?;
    tokio::spawn(async move {
        while let Some(msg) = roi_lanes_sub.next().await {
            let mut stats = lane_stats_clone.lock().unwrap();
            for (i, lane) in msg.lane.iter().enumerate() {
                if i < stats.len() {
                    stats[i].update(&lane.point);
                }
            }
        }
    });

    // Vehicle State 구독
    let vehicle_state_clone = vehicle_state.clone();
    let mut vehicle_state_sub =
        node.subscribe::<VehicleOutput>("/ego/vehicle_state", qos.clone())?;
    tokio::spawn(async move {
        while let Some(msg) = vehicle_state_sub.next().await {
            let mut state = vehicle_state_clone.lock().unwrap();
            state.update(&msg);
        }
    });

    // Clicked Point 구독
    let mut clicked_point_sub =
        node.subscribe::<geometry_msgs::msg::PointStamped>("/clicked_point", qos.clone())?;
    tokio::spawn(async move {
        while let Some(msg) = clicked_point_sub.next().await {
            println!(
                "Clicked Point: x={:.4}, y={:.4}, z={:.4}",
                msg.point.x, msg.point.y, msg.point.z
            );
        }
    });

    // Vehicle Command 구독
    let mut vehicle_command_sub =
        node.subscribe::<VehicleInput>("/ego/vehicle_command", qos.clone())?;
    tokio::spawn(async move {
        while let Some(msg) = vehicle_command_sub.next().await {
            println!(
                "Vehicle Command: steering={:.4}, accel={:.4}, brake={:.4}",
                msg.steering, msg.accel, msg.brake
            );
        }
    });

    // 주기적 데이터 출력
    let lane_stats_clone = lane_stats.clone();
    let vehicle_state_clone = vehicle_state.clone();
    let mut interval = interval(Duration::from_millis(100));
    tokio::spawn(async move {
        loop {
            interval.tick().await;

            // Lane Stats 출력
            let stats = lane_stats_clone.lock().unwrap();
            for (i, stat) in stats.iter().enumerate() {
                if stat.last_update.elapsed() < Duration::from_secs(1) {
                    println!(
                        "Lane {}: Avg Point: x={:.4}, y={:.4}, z={:.4}",
                        i, stat.avg_x, stat.avg_y, stat.avg_z
                    );
                }
            }

            // Vehicle State 출력
            let state = vehicle_state_clone.lock().unwrap();
            if state.last_update.elapsed() < Duration::from_secs(1) {
                println!(
                    "Vehicle State: id={}, x={:.4}, y={:.4}, yaw={:.4}, velocity={:.4}",
                    state.id, state.x, state.y, state.yaw, state.velocity
                );
            }
        }
    });

    loop {
        node.spin_once(Duration::from_millis(100));
    }
}
