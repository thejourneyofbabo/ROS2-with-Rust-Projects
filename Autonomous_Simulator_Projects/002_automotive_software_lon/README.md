# 자율주행 알고리즘 구현 상세 설명

이 프로젝트는 ROS2를 기반으로 한 자율주행 알고리즘의 구현입니다. 주요 기능으로는 속도 제어, 경로 추종, 그리고 주행 정보 발행 등이 있습니다.

## 코드 구조 및 주요 변경 사항

### 1. 클래스 선언 및 생성자

기존 코드:

```cpp
class AutonomousDriving : public rclcpp::Node {
public:
    AutonomousDriving(const std::string &node_name, const double &loop_rate,
                      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    // ...
};
```

수정된 코드:

```cpp
class AutonomousDriving : public rclcpp::Node {
public:
    AutonomousDriving(const std::string &node_name, const double &loop_rate,
                      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node(node_name, options),
          target_speeds_{30, 60, 90, 70, 50, 30, 100, 50, 100},
          current_speed_index_(0),
          last_speed_change_time_(this->now()),
          pid_last_time(this->now())
    {
        // ...
    }
    // ...
};
```

설명:
- `rclcpp::Node`를 상속받아 ROS2 노드를 생성합니다.
- 생성자에서 초기화 리스트를 사용하여 새로운 멤버 변수들을 초기화합니다.
- `this->now()`는 ROS2의 현재 시간을 얻는 메서드입니다.

### 2. 퍼블리셔 및 서브스크라이버 추가

기존 코드:

```cpp
// Publishers
p_vehicle_command_ = this->create_publisher<ad_msgs::msg::VehicleInput>(
    "vehicle_command", qos_profile);
p_driving_way_ = this->create_publisher<ad_msgs::msg::PolyfitLaneData>(
    "driving_way", qos_profile);
```

추가된 코드:

```cpp
// Additional Publishers
p_limit_speed_data_ = this->create_publisher<std_msgs::msg::Float64>("limit_speed_data", 10);
p_ego_velocity_ = this->create_publisher<std_msgs::msg::Float64>("ego/vehicle_state/velocity", 10);
```

설명:
- `create_publisher<T>(topic_name, qos_profile)`를 사용하여 퍼블리셔를 생성합니다.
- `T`는 메시지 타입을 나타냅니다 (예: `std_msgs::msg::Float64`).
- 토픽 이름과 QoS (Quality of Service) 프로파일을 지정합니다.

### 3. Run 메서드 수정

기존 코드:

```cpp
void AutonomousDriving::Run(const rclcpp::Time &current_time) {
    UpdateParameter();

    // Get subscribe variables
    mutex_vehicle_state_.lock();
    ad_msgs::msg::VehicleOutput vehicle_state = i_vehicle_state_;
    mutex_vehicle_state_.unlock();

    // ...
}
```

수정된 코드:

```cpp
void AutonomousDriving::Run(const rclcpp::Time &current_time) {
    UpdateParameter();

    // Get subscribe variables
    mutex_vehicle_state_.lock();
    ad_msgs::msg::VehicleOutput vehicle_state = i_vehicle_state_;
    mutex_vehicle_state_.unlock();

    // ...

    if (!param_use_manual_inputs_) {
        PID(current_time, target_speed, vehicle_state.velocity);

        if (computed_input >= 0.0) {
            vehicle_command.accel = computed_input;
            vehicle_command.brake = 0.0;
        } else {
            vehicle_command.accel = 0.0;
            vehicle_command.brake = -computed_input;
        }

        // Publish target speed
        auto target_speed_msg = std_msgs::msg::Float64();
        target_speed_msg.data = target_speed;
        p_limit_speed_data_->publish(target_speed_msg);

        // Publish current vehicle speed
        auto current_speed_msg = std_msgs::msg::Float64();
        current_speed_msg.data = vehicle_state.velocity;
        p_ego_velocity_->publish(current_speed_msg);

        RCLCPP_INFO(this->get_logger(), "Target speed index: %zu, Target speed: %f, Current speed: %f", 
                    current_speed_index_, target_speed, vehicle_state.velocity);
    }

    // ...
}
```

설명:
- `mutex`를 사용하여 스레드 안전성을 보장합니다.
- `PID` 함수를 호출하여 속도 제어 입력을 계산합니다.
- `publish` 메서드를 사용하여 메시지를 발행합니다.
- `RCLCPP_INFO`를 사용하여 로그 메시지를 출력합니다.

### 4. PID 컨트롤러 구현

새로 추가된 코드:

```cpp
void AutonomousDriving::PID(const rclcpp::Time& current_time, double target_value, double current_value) {
    double dt = (current_time - pid_last_time).seconds();

    if (dt <= 0.0) {
        dt = 0.01;
    }

    e = target_value - current_value;
    int_e += (dt * (e + e_prev) / 2.0);
    double dev_e = (e - e_prev) / dt;

    computed_input = param_pid_kp_ * e + param_pid_ki_ * int_e + param_pid_kd_ * dev_e;

    e_prev = e;
    pid_last_time = current_time;
}
```

설명:
- PID (비례-적분-미분) 제어 알고리즘을 구현합니다.
- `rclcpp::Time`을 사용하여 시간 간격을 계산합니다.
- 적분항은 사다리꼴 근사법을 사용하여 계산합니다.

### 5. 콜백 함수 수정

기존 코드:

```cpp
void AutonomousDriving::CallbackLimitSpeed(const std_msgs::msg::Float32::SharedPtr msg) {            
    mutex_limit_speed_.lock();
    i_limit_speed_ = msg->data;
    mutex_limit_speed_.unlock();
}
```

수정된 코드:

```cpp
void AutonomousDriving::CallbackLimitSpeed(const std_msgs::msg::Float32::SharedPtr msg) {            
    mutex_limit_speed_.lock();
    
    if ((this->now() - last_speed_change_time_).seconds() >= 10.0) {  // Change speed every 10 seconds
        current_speed_index_ = (current_speed_index_ + 1) % target_speeds_.size();
        last_speed_change_time_ = this->now();
        i_limit_speed_ = target_speeds_[current_speed_index_];
        RCLCPP_INFO(this->get_logger(), "Changing target speed to: %f", i_limit_speed_);
    } else {
        i_limit_speed_ = msg->data;
    }
    
    mutex_limit_speed_.unlock();
}
```

설명:
- `SharedPtr`을 사용하여 메시지를 효율적으로 전달받습니다.
- 10초마다 목표 속도를 변경하는 로직을 추가했습니다.
- `mutex`를 사용하여 스레드 안전성을 보장합니다.

## 주요 C++ 및 ROS2 개념

1. **클래스 상속**: `rclcpp::Node`를 상속받아 ROS2 노드를 구현합니다.
2. **스마트 포인터**: `SharedPtr`을 사용하여 메모리를 효율적으로 관리합니다.
3. **템플릿**: `create_publisher<T>` 등에서 템플릿을 사용하여 다양한 메시지 타입을 지원합니다.
4. **뮤텍스**: `std::mutex`를 사용하여 스레드 안전성을 보장합니다.
5. **람다 함수**: 타이머 콜백 등에서 람다 함수를 사용합니다.
6. **ROS2 파라미터**: `declare_parameter` 및 `get_parameter`를 사용하여 노드 파라미터를 관리합니다.
7. **ROS2 시간**: `this->now()`를 사용하여 현재 시간을 얻습니다.
8. **ROS2 로깅**: `RCLCPP_INFO` 등을 사용하여 로그 메시지를 출력합니다.

## 사용 방법

1. ROS2 환경을 설정합니다.
2. 이 패키지를 빌드합니다: `colcon build --packages-select [패키지 이름]`
3. 다음 명령어로 노드를 실행합니다: `ros2 run [패키지 이름] autonomous_driving`

## 파라미터 설정

런타임에 파라미터를 설정할 수 있습니다:

```
ros2 param set /autonomous_driving autonomous_driving/pid_kp 1.0
```

## 주의사항

- 이 알고리즘은 시뮬레이션 환경에서 테스트되었습니다. 실제 차량에 적용하기 전에 충분한 검증이 필요합니다.
- PID 게인 값은 차량의 특성에 따라 조정이 필요할 수 있습니다.

## 향후 개선 사항

- 장애물 회피 기능 추가
- 경로 계획 알고리즘 개선
- 실시간 환경 인식 기능 강화

