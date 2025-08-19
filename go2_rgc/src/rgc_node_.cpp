#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "unitree_go/msg/low_cmd.hpp"
#include "go2_rgc/go2_rgc.hpp"

using std::placeholders::_1;
using lowCmd = unitree_go::msg::LowCmd;

class RGCNode : public rclcpp::Node
{
public:
  RGCNode()
    : Node("rgc_node"), ready_(false), lifted_(false)
  {
    // Parameters
    this->declare_parameter("contact_frames", std::vector<std::string>{"FR_foot", "FL_foot", "RR_foot", "RL_foot"});
    this->declare_parameter("urdf_path", "/tmp/go2.urdf");
    this->declare_parameter("horizon", 10);
    this->declare_parameter("z_threshold", 0.25);  // altura mínima para considerar como "em pé"

    // Load parameters
    auto contact_frames = this->get_parameter("contact_frames").as_string_array();
    auto urdf_path = this->get_parameter("urdf_path").as_string();
    int horizon = this->get_parameter("horizon").as_int();

    // Init RGC model
    rgc_model_ = std::make_shared<go2_rgc::RGCModel>(urdf_path, horizon, contact_frames);

    // Subscribers
    // joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    //   "/joint_states", 10, std::bind(&RGCNode::joint_callback, this, _1));
        lowstate_subscriber_ = get_node()->create_subscription<lowStates>(
        "/lowstate", rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<lowStates> msg) -> void
        {
            std::lock_guard<std::mutex> lock(this->mutex_controller);

            for (auto index{0}; index < 12; index++)
            {
                _q[index] = msg->motor_state[index].q;
                _qd[index] = msg->motor_state[index].dq;
            }
            _lowTick = msg->tick;
        });


    // pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    //   "/go2/pose", 10, std::bind(&RGCNode::pose_callback, this, _1));

    // Publisher
    // cmd_pub_ = this->create_publisher<lowCmd>("/go2_jointcontroller/JointControllerReferences", 10);

    // Timer
    timer_ = this->create_wall_timer(5ms, std::bind(&RGCNode::publish_command, this));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<lowCmd>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<go2_rgc::RGCModel> rgc_model_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;
  Eigen::Vector3d r_;
  Eigen::Quaterniond Q_;
  bool ready_;
  bool lifted_;

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() != 12 || msg->velocity.size() != 12) return;

    q_ = Eigen::VectorXd::Zero(19);
    v_ = Eigen::VectorXd::Zero(18);

    // Free-flyer is identity / zero
    q_.segment(7, 12) = Eigen::Map<const Eigen::VectorXd>(msg->position.data(), 12);
    v_.segment(6, 12) = Eigen::Map<const Eigen::VectorXd>(msg->velocity.data(), 12);

    if (ready_)
      rgc_model_->updateState(q_, v_, r_, Q_);
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    r_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Q_ = Eigen::Quaterniond(
      msg->pose.orientation.w,
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z);
    ready_ = true;
  }

  void publish_command()
  {
    // Atualiza o pinocchio 

    // passa _data e _model go2_rgc.cpp
    if (!ready_ || lifted_) return;

    auto z_threshold = this->get_parameter("z_threshold").as_double();

    if (r_.z() > z_threshold)
    {
      lifted_ = true;
      RCLCPP_INFO(this->get_logger(), "Robot is already upright.");
      return;
    }

    // Send stand-up reference position
    lowCmd cmd_msg;
    float targetPos[12] = {
      0.0, 0.80, -1.36,
      0.0, 0.80, -1.36,
      0.0, 0.80, -1.36,
      0.0, 0.80, -1.36
    };

    for (int i = 0; i < 12; ++i)
    {
      cmd_msg.motor_cmd[i].q = targetPos[i];
      cmd_msg.motor_cmd[i].dq = 0;
      cmd_msg.motor_cmd[i].kp = 50.0;
      cmd_msg.motor_cmd[i].kd = 0.7;
      cmd_msg.motor_cmd[i].tau = 0;
    }

    cmd_pub_->publish(cmd_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RGCNode>());
  rclcpp::shutdown();
  return 0;
}
