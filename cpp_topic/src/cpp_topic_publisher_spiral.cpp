// #include <chrono>
// #include <memory>
// #include "rclcpp/rclcpp.hpp"
// #include <unitree_go/msg/low_state.hpp>
// #include "unitree_go/msg/low_cmd.hpp"

// using namespace std::chrono_literals;
// using lowCmd = unitree_go::msg::LowCmd;

// class MinimalPublisher : public rclcpp::Node
// {
// public:
//     MinimalPublisher()
//         : Node("low_cmd"), motion_time(0), rate_count(0), toggle_pos(false)
//     {
//         publisher_ = this->create_publisher<lowCmd>("/go2_jointcontroller/JointControllerReferences", 1);
//         // publisher_ = this->create_publisher<lowCmd>("/go2_actuator/LowCommands", 1);

//         // Começa na posição inicial
//         std::copy(std::begin(_targetPos_1), std::end(_targetPos_1), std::begin(_desPos));

//         timer_ = this->create_wall_timer(100ms, std::bind(&MinimalPublisher::publish_message, this));
//     }

// private:
//     void publish_message()
//     {
//         auto low_cmd = lowCmd();
//         motion_time++;

//         // Normalização do tempo para interpolação (0 a 1)
//         double rate = std::min(1.0, rate_count / 50.0);
//         rate_count++;

//         // Interpola entre as posições
//         for (int j = 0; j < 12; j++)
//         {
//             low_cmd.motor_cmd[j].q = _desPos[j];
//             low_cmd.motor_cmd[j].dq = 0;
//             low_cmd.motor_cmd[j].kp = 30.0;
//             low_cmd.motor_cmd[j].kd = 3;
//             low_cmd.motor_cmd[j].tau = 0;
//         }

//         // Interpolação entre as posições (1 -> 2 ou 2 -> 1) com base no valor de `toggle_pos`
//         for (int i = 0; i < 12; i++)
//         {
//             low_cmd.motor_cmd[i].q = jointLinearInterpolation(_startPos[i], _desPos[i], rate);
//         }

//         low_cmd.reserve = mode;
//         publisher_->publish(low_cmd);

//         // Alternância entre posições a cada 50 iterações
//         if (rate >= 1.0)
//         {
//             toggle_pos = !toggle_pos;

//             // Alterna entre _targetPos_1 e _targetPos_2 com base na direção da interpolação
//             if (toggle_pos)
//             {
//                 std::copy(std::begin(_targetPos_1), std::end(_targetPos_1), std::begin(_desPos));   // Interpolação de 1 para 2
//                 std::copy(std::begin(_targetPos_2), std::end(_targetPos_2), std::begin(_startPos)); // Inicia a interpolação da 2 para 1
//             }
//             else
//             {
//                 std::copy(std::begin(_targetPos_2), std::end(_targetPos_2), std::begin(_desPos));   // Interpolação de 2 para 1
//                 std::copy(std::begin(_targetPos_1), std::end(_targetPos_1), std::begin(_startPos)); // Inicia a interpolação de 1 para 2
//             }

//             rate_count = 0; // Reinicia interpolação
//             cout2_ += 1;
//             if (cout2_ == 4)
//             {
//                 mode = 1; // Muda o modo para 2
//                 cout2_ = 0;
//             }
//         }
//     }

//     double jointLinearInterpolation(double q0, double qf, double rate)
//     {
//         return q0 + rate * (qf - q0);
//     }

//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<lowCmd>::SharedPtr publisher_;
//     bool toggle_pos = false;
//     int motion_time, rate_count;
//     float _desPos[12]; // Agora, _desPos é um array de 12 posições.

//     int cout2_ = 0;
//     uint32_t mode = 1;

//     // float _startPos[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};
//     // float _targetPos_1[12] = {0.0, 1.2, -2.05, 0.5, 0.8, -1.55, -0.2, 0.8, -1.55, 0.2, 1.0, -2.1};
//     // float _targetPos_2[12] = {0.2, 1.5, -1.8, 0.3, 1.2, -1.25, -0.1, 1.0, -1.2, 0.3, 1.0, -1.6};

//     float _startPos[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};
//     float _targetPos_1[12] = {0.0, 1.36, -2.5, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};
//     float _targetPos_2[12] = {0.0, 1.36, -1.0, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

//     // float _targetPos_1[12] = {
//     //     0.2, 1.3, -2.3,  // FR leg
//     //     -0.2, 1.3, -2.3, // FL leg
//     //     0.2, 1.3, -2.3,  // RR leg
//     //     -0.2, 1.3, -2.3  // RL leg
//     // };
    
//     // float _targetPos_2[12] = {
//     //     0.0, 1.0, -1.8,  // FR leg
//     //     0.0, 1.0, -1.8,  // FL leg
//     //     0.0, 1.0, -1.8,  // RR leg
//     //     0.0, 1.0, -1.8   // RL leg
//     // };
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MinimalPublisher>());
//     rclcpp::shutdown();
//     return 0;
// }

#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"

using namespace std::chrono_literals;
using lowCmd = unitree_go::msg::LowCmd;

class GaitPublisher : public rclcpp::Node
{
public:
    GaitPublisher()
        : Node("go2_gait_publisher")
    {
        publisher_ = this->create_publisher<lowCmd>("/go2_jointcontroller/JointControllerReferences", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&GaitPublisher::publish_command, this));
        start_time_ = this->now();
    }

private:
    void publish_command()
    {
        rclcpp::Time time_now = this->now();
        double t = (time_now - start_time_).seconds();

        lowCmd msg;

        for (int i = 0; i < 4; i++) // Four legs
        {
            int hip = i * 3 + 0;
            int thigh = i * 3 + 1;
            int knee = i * 3 + 2;

            double phase = (i % 2 == 0) ? 0.0 : M_PI; // Diagonal leg pairs in opposite phase

            // Hip: swing forward/backward
            msg.motor_cmd[hip].q = 0.3 * sin(2 * M_PI * 0.5 * t + phase); // +/- 0.3 rad
            // Thigh: fixed downward
            msg.motor_cmd[thigh].q = 1.2;
            // Knee: bend/unbend
            msg.motor_cmd[knee].q = -2.0 + 0.2 * sin(2 * M_PI * 0.5 * t + phase);

            for (int j = 0; j < 3; j++) {
                int idx = i * 3 + j;
                msg.motor_cmd[idx].dq = 0.0;
                msg.motor_cmd[idx].kp = 30.0;
                msg.motor_cmd[idx].kd = 3.0;
                msg.motor_cmd[idx].tau = 0.0;
            }
        }

        msg.reserve = 1; // Mode (if used)
        publisher_->publish(msg);
    }

    rclcpp::Publisher<lowCmd>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaitPublisher>());
    rclcpp::shutdown();
    return 0;
}

