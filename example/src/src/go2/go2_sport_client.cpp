/**********************************************************************
 Copyright...
***********************************************************************/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

#include "common/ros2_sport_client.h"

#define TOPIC_HIGHSTATE "lf/sportmodestate"  // 필요시 launch에서 /sportmodestate로 remap

using namespace std::chrono_literals;

class Go2SportClientNode : public rclcpp::Node {
public:
  explicit Go2SportClientNode(int test_mode)
  : Node("go2_sport_client_node"),
    sport_client_(this),
    test_mode_(test_mode)
  {
    // 고수준 상태 구독(선택)
    sub_state_ = this->create_subscription<unitree_go::msg::SportModeState>(
      TOPIC_HIGHSTATE, rclcpp::QoS(10),
      [this](const unitree_go::msg::SportModeState::SharedPtr msg){ state_ = *msg; });

    // cmd_vel 구독
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){
        std::scoped_lock lk(cmd_mtx_);
        last_twist_ = *msg;
        last_cmd_time_ = this->now();
        have_cmd_ = true;
      });

    // 100ms 제어 루프
    timer_ = this->create_wall_timer(100ms, [this]{ this->ControlLoop(); });

    // 초기자세(일어서기/밸런스 등) 한 번 수행
    t1_ = std::thread([this]{
      std::this_thread::sleep_for(500ms);
      this->InitialPosture();
    });
  }

  ~Go2SportClientNode() override {
    if (t1_.joinable()) t1_.join();
    // 안전 정지
    sport_client_.StopMove(req_);
  }

private:
  // 초기 자세 설정
  void InitialPosture() {
    switch (test_mode_) {
      case 0: // NORMAL_STAND
      case 4: // STAND_UP
        sport_client_.StandUp(req_); break;
      case 1: // BALANCE_STAND
        sport_client_.BalanceStand(req_); break;
      case 5: // DAMP
        sport_client_.Damp(req_); break;
      case 6: // RECOVERY_STAND
        sport_client_.RecoveryStand(req_); break;
      case 7: // SIT
        sport_client_.Sit(req_); break;
      case 8: // RISE_SIT
        sport_client_.RiseSit(req_); break;
      case 3: // STAND_DOWN
        sport_client_.StandDown(req_); break;
      default:
        // MOVE/STOP은 루프에서 처리
        break;
    }
  }

  // 주기 제어: /cmd_vel → Move, 타임아웃 시 Stop
  void ControlLoop() {
    const auto now = this->now();
    bool timeout = false;

    {
      std::scoped_lock lk(cmd_mtx_);
      if (!have_cmd_) timeout = true;
      else timeout = (now - last_cmd_time_).seconds() > deadman_sec_;
    }

    if (timeout) {
      sport_client_.StopMove(req_);
      return;
    }

    geometry_msgs::msg::Twist tw;
    {
      std::scoped_lock lk(cmd_mtx_);
      tw = last_twist_;
    }

    // 속도 제한 (필요시 조정)
    const double vx = clamp(tw.linear.x,  -vx_max_,  vx_max_);
    const double vy = clamp(tw.linear.y,  -vy_max_,  vy_max_);
    const double wz = clamp(tw.angular.z, -wz_max_,  wz_max_);

    sport_client_.Move(req_, static_cast<float>(vx),
                             static_cast<float>(vy),
                             static_cast<float>(wz));
  }

  static double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
  }

  // 멤버
  unitree_go::msg::SportModeState state_;
  SportClient sport_client_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sub_state_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::TimerBase::SharedPtr timer_;
  unitree_api::msg::Request req_;
  std::thread t1_;
  int test_mode_{0};

  std::mutex cmd_mtx_;
  geometry_msgs::msg::Twist last_twist_{};
  rclcpp::Time last_cmd_time_{0,0,RCL_ROS_TIME};
  bool have_cmd_{false};

  // 제한/데드맨 파라미터
  const double vx_max_ = 0.6;    // m/s
  const double vy_max_ = 0.3;    // m/s
  const double wz_max_ = 0.8;    // rad/s
  const double deadman_sec_ = 0.5;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int test_mode = 0;
  if (argc >= 2) test_mode = std::atoi(argv[1]); // 0: StandUp 기본

  auto node = std::make_shared<Go2SportClientNode>(test_mode);
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
