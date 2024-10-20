use futures::StreamExt;
use r2r::ad_msgs::msg::{LanePointDataArray, VehicleInput, VehicleOutput};
use r2r::geometry_msgs;
use r2r::QosProfile;
use rand::rngs::StdRng;
use rand::Rng;
use rand::SeedableRng;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::time::interval;

// 차선 통계를 저장하는 구조체
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

// 차량 상태를 저장하는 구조체
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

// 차량 제어 명령을 저장하는 구조체
#[derive(Debug, Clone)]
struct VehicleControl {
    steering: f64,
    accel: f64,
    brake: f64,
}

impl VehicleControl {
    fn new() -> Self {
        Self {
            steering: 0.0,
            accel: 0.0,
            brake: 0.0,
        }
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "rust_multi_subscriber_node", "")?;
    let qos = QosProfile::default();
    println!("Node created: rust_multi_subscriber_node");

    // 공유 데이터 구조 초기화
    let lane_stats = Arc::new(Mutex::new(vec![LaneStats::new(), LaneStats::new()]));
    let vehicle_state = Arc::new(Mutex::new(VehicleState::new()));
    let vehicle_control = Arc::new(Mutex::new(VehicleControl::new()));

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

    // 차량 제어 로직
    let vehicle_state_clone = vehicle_state.clone();
    let vehicle_control_clone = vehicle_control.clone();
    let vehicle_command_pub =
        node.create_publisher::<VehicleInput>("/ego/vehicle_command", qos.clone())?;
    tokio::spawn(async move {
        let mut interval = interval(Duration::from_millis(100));
        let mut rng = StdRng::from_entropy();
        loop {
            interval.tick().await;

            let state = vehicle_state_clone.lock().unwrap();
            let mut control = vehicle_control_clone.lock().unwrap();

            // 속도 조절 (5~9 사이로 유지)
            if state.velocity < 5.0 {
                control.accel = 30.5;
                control.brake = 0.0;
            } else if state.velocity > 9.0 {
                control.accel = 0.0;
                control.brake = 10.0;
            } else {
                control.accel = 0.05;
                control.brake = 0.0;
            }

            // 조향 조절 (약간의 랜덤성 추가)
            control.steering += rng.gen_range(-0.02..0.02);
            control.steering = control.steering.clamp(-0.5, 0.5);

            let msg = VehicleInput {
                steering: control.steering,
                accel: control.accel,
                brake: control.brake,
            };
            vehicle_command_pub.publish(&msg).unwrap();

            // 디버그 출력
            println!(
                "Published command: steering={:.4}, accel={:.4}, brake={:.4}, velocity={:.4}",
                control.steering, control.accel, control.brake, state.velocity
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

    // 메인 루프
    loop {
        node.spin_once(Duration::from_millis(100));
    }
}
