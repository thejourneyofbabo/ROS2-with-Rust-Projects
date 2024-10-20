use futures::StreamExt;
use nalgebra::{DMatrix, DVector, SVD};
use r2r::ad_msgs::msg::{LanePointDataArray, VehicleInput, VehicleOutput};
use r2r::geometry_msgs;
use r2r::QosProfile;
use rand::rngs::StdRng;
use rand::Rng;
use rand::SeedableRng;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::time::interval;

#[derive(Debug, Clone)]
struct Point {
    x: f64,
    y: f64,
    z: f64,
}

#[derive(Debug, Clone)]
struct Lane {
    a: f64,
    b: f64,
    c: f64,
    d: f64,
}

impl Lane {
    fn calculate_center(&self, other: &Lane) -> Lane {
        Lane {
            a: (self.a + other.a) / 2.0,
            b: (self.b + other.b) / 2.0,
            c: (self.c + other.c) / 2.0,
            d: (self.d + other.d) / 2.0,
        }
    }
}

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

    fn update(&mut self, points: &[Point]) {
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

struct PIDController {
    kp: f64,
    ki: f64,
    kd: f64,
    previous_error: f64,
    integral: f64,
}

impl PIDController {
    fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            previous_error: 0.0,
            integral: 0.0,
        }
    }

    fn compute(&mut self, error: f64, dt: f64) -> f64 {
        self.integral += error * dt;
        let derivative = (error - self.previous_error) / dt;
        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        self.previous_error = error;
        output
    }
}

fn detect_lanes(points: &[Point]) -> Result<Lane, Box<dyn std::error::Error>> {
    let n = points.len();
    if n < 4 {
        return Err("Not enough points to detect lanes".into());
    }

    let mut x_matrix = DMatrix::zeros(n, 4);
    let mut y_vector = DVector::zeros(n);

    for (i, point) in points.iter().enumerate() {
        x_matrix[(i, 0)] = point.x.powi(3);
        x_matrix[(i, 1)] = point.x.powi(2);
        x_matrix[(i, 2)] = point.x;
        x_matrix[(i, 3)] = 1.0;
        y_vector[i] = point.y;
    }

    let svd = SVD::new(x_matrix, true, true);
    let pseudo_inverse = svd
        .solve(&y_vector, 1e-10)
        .map_err(|_| "Failed to compute pseudo-inverse")?;

    Ok(Lane {
        a: pseudo_inverse[0],
        b: pseudo_inverse[1],
        c: pseudo_inverse[2],
        d: pseudo_inverse[3],
    })
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "rust_multi_subscriber_node", "")?;
    let qos = QosProfile::default();
    println!("Node created: rust_multi_subscriber_node");

    let lane_stats = Arc::new(Mutex::new(vec![LaneStats::new(), LaneStats::new()]));
    let vehicle_state = Arc::new(Mutex::new(VehicleState::new()));
    let vehicle_control = Arc::new(Mutex::new(VehicleControl::new()));
    let detected_lanes = Arc::new(Mutex::new(vec![
        Lane {
            a: 0.0,
            b: 0.0,
            c: 0.0,
            d: 0.0,
        },
        Lane {
            a: 0.0,
            b: 0.0,
            c: 0.0,
            d: 0.0,
        },
    ]));

    let lane_stats_clone = lane_stats.clone();
    let detected_lanes_clone = detected_lanes.clone();
    let mut roi_lanes_sub = node.subscribe::<LanePointDataArray>("/ego/ROI_lanes", qos.clone())?;
    tokio::spawn(async move {
        while let Some(msg) = roi_lanes_sub.next().await {
            let mut stats = lane_stats_clone.lock().unwrap();
            let mut lanes = detected_lanes_clone.lock().unwrap();
            for (i, lane) in msg.lane.iter().enumerate() {
                if i < stats.len() {
                    let points: Vec<Point> = lane
                        .point
                        .iter()
                        .map(|p| Point {
                            x: p.x,
                            y: p.y,
                            z: p.z,
                        })
                        .collect();
                    stats[i].update(&points);
                    if let Ok(detected) = detect_lanes(&points) {
                        lanes[i] = detected;
                    }
                }
            }
        }
    });

    let vehicle_state_clone = vehicle_state.clone();
    let mut vehicle_state_sub =
        node.subscribe::<VehicleOutput>("/ego/vehicle_state", qos.clone())?;
    tokio::spawn(async move {
        while let Some(msg) = vehicle_state_sub.next().await {
            let mut state = vehicle_state_clone.lock().unwrap();
            state.update(&msg);
        }
    });

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

    let vehicle_state_clone = vehicle_state.clone();
    let vehicle_control_clone = vehicle_control.clone();
    let detected_lanes_clone = detected_lanes.clone();
    let vehicle_command_pub =
        node.create_publisher::<VehicleInput>("/ego/vehicle_command", qos.clone())?;

    let mut pid_controller = PIDController::new(0.56, 0.05, 0.05);
    let mut last_time = Instant::now();

    tokio::spawn(async move {
        let mut interval = interval(Duration::from_millis(100));
        loop {
            interval.tick().await;

            let state = vehicle_state_clone.lock().unwrap();
            let mut control = vehicle_control_clone.lock().unwrap();
            let lanes = detected_lanes_clone.lock().unwrap();

            if lanes.len() >= 2 {
                let center_lane = lanes[0].calculate_center(&lanes[1]);

                // 속도 제어
                if state.velocity < 60.0 {
                    control.accel = 1.0;
                    control.brake = 0.0;
                } else if state.velocity > 120.0 {
                    control.accel = 0.0;
                    control.brake = 0.5;
                } else {
                    control.accel = 0.5;
                    control.brake = 0.0;
                }

                // PID 기반 조향 제어
                let lookahead: f64 = 10.0;
                let desired_y = center_lane.a * lookahead.powi(3)
                    + center_lane.b * lookahead.powi(2)
                    + center_lane.c * lookahead
                    + center_lane.d;
                let current_y = state.y;
                let error = desired_y - current_y;

                let now = Instant::now();
                let dt = (now - last_time).as_secs_f64();
                last_time = now;

                let pid_output = pid_controller.compute(error, dt);
                control.steering = pid_output.clamp(-0.5, 0.5);

                let msg = VehicleInput {
                    steering: control.steering,
                    accel: control.accel,
                    brake: control.brake,
                };
                vehicle_command_pub.publish(&msg).unwrap();

                println!(
                    "Published command: steering={:.4}, accel={:.4}, brake={:.4}, velocity={:.4}, error={:.4}",
                    control.steering, control.accel, control.brake, state.velocity, error
                );
            } else {
                println!("Not enough lanes detected for center line calculation");
            }
        }
    });

    let lane_stats_clone = lane_stats.clone();
    let vehicle_state_clone = vehicle_state.clone();
    let detected_lanes_clone = detected_lanes.clone();
    let mut interval = interval(Duration::from_millis(100));
    tokio::spawn(async move {
        loop {
            interval.tick().await;

            let stats = lane_stats_clone.lock().unwrap();
            for (i, stat) in stats.iter().enumerate() {
                if stat.last_update.elapsed() < Duration::from_secs(1) {
                    println!(
                        "Lane {}: Avg Point: x={:.4}, y={:.4}, z={:.4}",
                        i, stat.avg_x, stat.avg_y, stat.avg_z
                    );
                }
            }

            let state = vehicle_state_clone.lock().unwrap();
            if state.last_update.elapsed() < Duration::from_secs(1) {
                println!(
                    "Vehicle State: id={}, x={:.4}, y={:.4}, yaw={:.4}, velocity={:.4}",
                    state.id, state.x, state.y, state.yaw, state.velocity
                );
            }

            let lanes = detected_lanes_clone.lock().unwrap();
            for (i, lane) in lanes.iter().enumerate() {
                println!(
                    "Detected Lane {}: a={:.4}, b={:.4}, c={:.4}, d={:.4}",
                    i, lane.a, lane.b, lane.c, lane.d
                );
            }
        }
    });

    loop {
        node.spin_once(Duration::from_millis(100));
    }
}
