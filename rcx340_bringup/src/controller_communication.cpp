#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "yk400xe_interfaces/msg/move.hpp"
#include "yk400xe_interfaces/srv/move_trajectory.hpp"

#include "rcx340_bringup/telnet.hpp"
#include "rcx340_bringup/errors.hpp"

#include "rcx340_bringup/controller.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <math.h>
#include <deque>
#include <unordered_map>
#include <functional>
#include <chrono>
#include <sstream>

#include <iostream>

class ControllerCom : public rclcpp::Node{
    private:
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_jointStatesPublisher;
        rclcpp::TimerBase::SharedPtr m_jointStateTimer;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_efStatePublisher; 

        std::string m_ip;
        int m_port;

        TelnetCommunication* m_telnet;
        
        std::vector<std::string> m_jointsNames;

        std::deque<int> m_pendingCommands;

        std::unordered_map<int, std::function<void(const std::string&)>> m_handlerMap;

        Controller* m_controller;

        rclcpp::Service<yk400xe_interfaces::srv::MoveTrajectory>::SharedPtr m_moveService;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_servoService;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_alarmResetService;

    public:
        ControllerCom() : 
            Node("controller_communication_node"){
            
            m_handlerMap = {
                {1, [this](const std::string& s){handle_joints_state(s);}},
                {2, [this](const std::string& s){handle_ef_state(s);}},
                {3, [this](const std::string& s){handle_alarm_status(s);}},
                {4, [this](const std::string& s){handle_mspeed_status(s);}},
                {5, [this](const std::string& s){handle_return_to_origin_status(s);}},
                {6, [this](const std::string& s){handle_motor_status(s);}},
                {7, [this](const std::string& s){handle_servo_status(s);}}
            };

            this->declare_parameter<std::string>(
                "controller_ip","192.168.0.2");

            this->declare_parameter<int>(
                "controller_port",23);

            this->declare_parameter<std::vector<std::string>>(
                "joints_names", 
                std::vector<std::string>{"J1","J2","J3","J4"});

           this->declare_parameter<std::vector<double>>(
                "joints_ranges",
                {   
                // encoder min  encoder max   range min     range max
                    -385506,      380555,    -2.3038346,    2.3038346,
                    -437972,      439198,    -2.6179939,    2.6179939,
                    -1707,        250000,           0.0,       0.150,
                    -245760,      245760,    -6.2831853,    6.2831853,
                    -260.0,       450.0,     -6.2831853,    6.2831853
                }
            );

            this->get_parameter("controller_ip",m_ip);
            this->get_parameter("controller_port",m_port);
            this->get_parameter("joints_names",m_jointsNames);

            std::vector<double> flat;
            this->get_parameter("joints_ranges", flat);

            m_jointStatesPublisher =
                this->create_publisher<sensor_msgs::msg::JointState>(
                "/joint_states", 10);

            m_efStatePublisher =
                this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/ef_states", 10);
            
            // Service the send command in cartesian coordinates
            m_moveService = this->create_service<yk400xe_interfaces::srv::MoveTrajectory>(
                "/command/move",
                std::bind(  &ControllerCom::handle_move_service,
                            this,
                            std::placeholders::_1,
                            std::placeholders::_2
                        )
            );

            // Service to turn on/off the servos
            m_servoService = this->create_service<std_srvs::srv::SetBool>(
                "/command/servo",
                std::bind(  &ControllerCom::handle_set_servo_service,
                            this,
                            std::placeholders::_1,
                            std::placeholders::_2
                        )
            );

            // Service to reset the alarm
            m_alarmResetService = this->create_service<std_srvs::srv::Empty>(
                "/command/alarm_reset",
                std::bind(  &ControllerCom::handle_alarm_reset_service,
                            this,
                            std::placeholders::_1,
                            std::placeholders::_2
                        )
            );

            m_jointStateTimer = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&ControllerCom::timer_callback, this)
            );

            m_controller = new Controller(m_ip,m_port,flat,
                [this](const std::string& msg) {
                    this->process_message(msg);
                });

            RCLCPP_INFO(this->get_logger(), 
                "Communication node started"
            );
        }

    private:
        // Reset alarm
        void handle_alarm_reset_service(
            const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
            std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/){
            m_controller->alarm_reset();
        }

        // Set the servos on/off
        void handle_set_servo_service(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> /*response*/){

            if(request->data){
                RCLCPP_INFO(
                    this->get_logger(),
                    "Starting Servos"
                );
                m_controller->servo_on();
            }
            else{
                RCLCPP_INFO(
                    this->get_logger(),
                    "Stoping Servos"
                );
                m_controller->servo_off();
            }
        }

        // Send XY command to controller
        void handle_move_service(
                const std::shared_ptr<yk400xe_interfaces::srv::MoveTrajectory::Request> request,
                std::shared_ptr<yk400xe_interfaces::srv::MoveTrajectory::Response> /*response*/){

            if(true) // check if the mqnuql lock or servo off TODO
                m_controller->move_trajectory(request->move_cmds);

            RCLCPP_INFO(
                this->get_logger(),
                "Received request Move request : %ld Point(s) trajectory",
                request->move_cmds.size()
            );
            
        }

        // Send command to get joints states and end effectors position
        void timer_callback(){
            m_controller->where();
            m_pendingCommands.push_back(1);

            m_controller->where_xy();
            m_pendingCommands.push_back(2);

            // m_controller->alarm_status();
            // m_pendingCommands.push_front(3);

            // m_controller->mspeed_status();
            // m_pendingCommands.push_front(4);

            // m_controller->return_to_origin_status();
            // m_pendingCommands.push_front(5);

            // m_controller->motor_status();
            // m_pendingCommands.push_front(6);

            // m_controller->servo_status();
            // m_pendingCommands.push_front(7);
        }


        void process_message(const std::string& msg){
            
            // Start or End of tasks, SETUP TODO
            if (msg=="OK" ||
                msg=="END" ||
                msg=="BEGIN" ||
                msg=="RUN" ||
                msg=="READY" ||
                msg.empty()) return;
            
            // Welcome message
            if (msg[0]=='W'){
                RCLCPP_INFO(
                    this->get_logger(), 
                    "\033[32m%s\033[0m",
                    msg.c_str()
                );
                return;
            }

            // Errors handling using the dictionnary from errors.hpp
            if (msg[0]=='N'){
                std::string error_code = msg.substr(3);
                const auto& dict = errorDictionary();

                auto it = dict.find(error_code);
                if (it != dict.end())
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "\033[38;5;202m%s :  %s\033[0m",
                        error_code.c_str(),
                        it->second.c_str()
                    );
                else
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "\033[38;5;202mUnknown error : %s\033[0m",
                        error_code.c_str()
                    );
                return;
            }

            // Unexpected message received
            if (m_pendingCommands.empty()) {
                RCLCPP_WARN(
                    this->get_logger(), 
                    "Message received without pending command '%s'", 
                    msg.c_str()
                );
                return;
            }
            

            int cmdID = m_pendingCommands.front();
            m_pendingCommands.pop_front();  

            m_handlerMap[cmdID](msg);
        }
        
        // Function to parse and publish joints state
        void handle_joints_state(const std::string& mes){
            std::array<int,4> pulse_positions;
            std::array<double,4> ros_positions;

            size_t pos ;
            size_t start = 0;

            for (int i=0; i<4; i++){
                pos = mes.find(' ',start);
                std::string state = mes.substr(start, pos);

                int value;
                std::stringstream ss(state);
                ss >> value;
                pulse_positions[i] = value;
                start = pos+1;
            }

            ros_positions = m_controller->remap_from_pulse(pulse_positions);
            std::vector<double> v(ros_positions.begin(), ros_positions.end());

            std_msgs::msg::Header header;
            header.stamp = this->now();
            header.frame_id = "base_link"; 

            sensor_msgs::msg::JointState joint_state_msg;
            joint_state_msg.header=header;
            joint_state_msg.name=m_jointsNames;
            joint_state_msg.position=v;            

            m_jointStatesPublisher->publish(joint_state_msg);
        }
        
        // Function to parse and publish end effector position
        void handle_ef_state(const std::string& mes){
            std::vector<double> efPositions;

            size_t pos ;
            size_t start = 0;

            for (int i=0; i<4; i++){
                pos = mes.find(' ',start);
                std::string state = mes.substr(start, pos);

                double value;
                std::stringstream ss(state);
                ss >> value;
                efPositions.push_back(value);
                start = pos+1;
            }
            
            geometry_msgs::msg::Point efPoint;
            efPoint.x = efPositions[0];
            efPoint.y = efPositions[1];
            efPoint.z = efPositions[2];

            geometry_msgs::msg::Quaternion efQuaternion;
            efQuaternion.z = efPositions[3];

            std_msgs::msg::Header efHeader = std_msgs::msg::Header();
            efHeader.frame_id = "std_coords_joint";
            efHeader.stamp = rclcpp::Clock().now(); 

            geometry_msgs::msg::Pose efPose;
            efPose.position = efPoint;
            efPose.orientation = efQuaternion;

            geometry_msgs::msg::PoseStamped efMsg;
            efMsg.header = efHeader;
            efMsg.pose = efPose;
            

            m_efStatePublisher->publish(efMsg);
        }

        // Function to handle alarm status
        void handle_alarm_status(const std::string& /*mes*/){
            //
            // RCLCPP_INFO(
                // this->get_logger(),
                // "Received alarm status"
            // );
        }
        
        // Function to hangle mspeed status
        void handle_mspeed_status(const std::string& /*mes*/){
            //
            // RCLCPP_INFO(
                // this->get_logger(),
                // "Received mspeed status"
            // );
        }

        // Function to hangle return to origin status
        void handle_return_to_origin_status(const std::string& /*mes*/){
            //
            // RCLCPP_INFO(
                // this->get_logger(),
                // "Received return to origin status"
            // );
        }

        // Function to hangle motor status
        void handle_motor_status(const std::string& /*mes*/){
            //
            // RCLCPP_INFO(
                // this->get_logger(),
                // "Received motor status"
            // );
        }

        // Function to hangle servo status
        void handle_servo_status(const std::string& /*mes*/){
            //
            // RCLCPP_INFO(
                // this->get_logger(),
                // "Received servo status"
            // );
        }

};

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ControllerCom>());
    rclcpp::shutdown();

    return 0;
}