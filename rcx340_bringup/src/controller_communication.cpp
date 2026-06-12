#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
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
#include <queue>
#include <unordered_map>
#include <functional>
#include <chrono>
#include <sstream>

class ControllerCom : public rclcpp::Node{
    private:
        rclcpp::TimerBase::SharedPtr m_jointStateTimer;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_jointStatesPublisher;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_efStatePublisher; 

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_alarmStatusPublisher; 
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_emergencyStatusPublisher; 
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr m_mspeedStatusPublisher; 
        rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr m_returnToOriginStatusPublisher; 
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr m_motorStatusPublisher; 
        rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr m_servoStatusPublisher; 
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr m_modeStatusPublisher; 

        std::string m_ip;
        int m_port;
        std::vector<std::string> m_jointsNames;
        double m_j4_offset;

        TelnetCommunication* m_telnet;

        std::queue<int> m_pendingCommands;

        bool m_busy;
        std::queue<std::shared_ptr<yk400xe_interfaces::srv::MoveTrajectory::Request>> m_pendingMoveCommands;

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
                {7, [this](const std::string& s){handle_servo_status(s);}},
                {8, [this](const std::string& s){handle_task1_status(s);}},
                {9, [this](const std::string& s){handle_mode_status(s);}},
                {10, [this](const std::string& s){handle_emergency_status(s);}}

            };

            this->declare_parameter<std::string>(
                "controller_ip","192.168.0.2");

            this->declare_parameter<int>(
                "controller_port",23);

            this->declare_parameter<std::vector<std::string>>(
                "joints_names", 
                std::vector<std::string>{"J1","J2","J3","J4"});

            this->declare_parameter<double>(
                "j4_offset",100.0);

            this->get_parameter("controller_ip",m_ip);
            this->get_parameter("controller_port",m_port);
            this->get_parameter("joints_names",m_jointsNames);
            this->get_parameter("j4_offset",m_j4_offset);

            m_jointStatesPublisher =
                this->create_publisher<sensor_msgs::msg::JointState>(
                "/joint_states", 10);

            m_efStatePublisher =
                this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/ee_state", 10);

            m_alarmStatusPublisher =
                this->create_publisher<std_msgs::msg::String>(
                "/status/alarm", 10);

            m_emergencyStatusPublisher =
                this->create_publisher<std_msgs::msg::Bool>(
                "/status/emergency_stop", 10);

            m_mspeedStatusPublisher =
                this->create_publisher<std_msgs::msg::Int8>(
                "/status/mspeed", 10);

            m_returnToOriginStatusPublisher =
                this->create_publisher<std_msgs::msg::Int8MultiArray>(
                "/status/return_to_origin", 10);

            m_motorStatusPublisher =
                this->create_publisher<std_msgs::msg::Int8>(
                "/status/motor", 10);

            m_servoStatusPublisher =
                this->create_publisher<std_msgs::msg::Int8MultiArray>(
                "/status/servo", 10);

            m_modeStatusPublisher =
                this->create_publisher<std_msgs::msg::Int8>(
                "/status/mode", 10);
            
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

            m_controller = new Controller(m_ip,m_port,m_j4_offset,
                [this](const std::string& msg) {
                    this->process_message(msg);
                });
            
            m_busy = false;

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

        void processNextMoveCommand(){
            if(m_pendingMoveCommands.empty()) return;

            auto request = m_pendingMoveCommands.front();
            m_pendingMoveCommands.pop();

            m_busy = true;
            m_controller->move_trajectory(request->move_cmds);
        }

        // Send XY command to controller
        void handle_move_service(
                const std::shared_ptr<yk400xe_interfaces::srv::MoveTrajectory::Request> request,
                std::shared_ptr<yk400xe_interfaces::srv::MoveTrajectory::Response> /*response*/){

            RCLCPP_INFO(
                this->get_logger(),
                "Received request Move request : %ld Point(s) trajectory",
                request->move_cmds.size()
            );

            if (m_busy){
                m_pendingMoveCommands.push(request);
            }
            else{
                m_controller->move_trajectory(request->move_cmds);
            }
            
        }

        // Send command to get joints states and end effectors position
        void timer_callback(){
            
            m_controller->where();
            m_pendingCommands.push(1);

            m_controller->where_xy();
            m_pendingCommands.push(2);

            m_controller->alarm_status();
            m_pendingCommands.push(3);

            m_controller->mspeed_status();
            m_pendingCommands.push(4);

            m_controller->return_to_origin_status();
            m_pendingCommands.push(5);

            m_controller->motor_status();
            m_pendingCommands.push(6);

            m_controller->servo_status();
            m_pendingCommands.push(7);

            m_controller->task1_status();
            m_pendingCommands.push(8);

            m_controller->mode_status();
            m_pendingCommands.push(9);

            m_controller->emergency_status();
            m_pendingCommands.push(10);
        }


        void process_message(const std::string& msg){
            
            // Display welcome message
            if (msg[0]=='W'){
                RCLCPP_INFO(
                    this->get_logger(), 
                    "\033[32m%s\033[0m",
                    msg.c_str()
                );
                return;
            }

            // Skipping messages
            else if (msg=="OK" ||
                msg=="END" ||
                msg=="BEGIN" ||
                msg=="RUN" ||
                msg=="READY" ||
                msg.empty()
            ) return;
            
            
            // Errors handling using the dictionnary from errors.hpp
            else if (msg[0]=='N'){
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
            
            // Response to a pending command, handle it
            int cmdID = m_pendingCommands.front();
            m_pendingCommands.pop();  

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
            efPoint.x = efPositions[0]/1000;
            efPoint.y = efPositions[1]/1000;
            efPoint.z = -efPositions[2]/1000;

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
        // gg.bb (list in errors.hpp)
        void handle_alarm_status(const std::string& mes){
            std::string error_txt;
            const auto& dict = errorDictionary();

            auto it = dict.find(mes);
            if (it != dict.end())
                error_txt = it->second.c_str();
                
            auto msg = std_msgs::msg::String();
            msg.data = mes + " : " + error_txt;

            m_alarmStatusPublisher->publish(msg);
        }

        // Function to handle alarm status
        // 0 - Normal operation
        // 1 - Emergency stop on
        void handle_emergency_status(const std::string& mes){

            auto msg = std_msgs::msg::Bool();

            if(mes[0] == '0') msg.data = false;
            else msg.data = true;

            m_emergencyStatusPublisher->publish(msg);
        }
        
        // Function to hangle mspeed status
        // m   --> 1 to 100 %
        void handle_mspeed_status(const std::string& mes){

            auto msg = std_msgs::msg::Int8();
            
            msg.data = static_cast<int8_t>(std::stoi(mes));

            m_mspeedStatusPublisher->publish(msg);
        }

        // Function to hangle return to origin status
        // all 1,2,3,4,5,6 but we ignore 5 and 6
        void handle_return_to_origin_status(const std::string& mes){

            auto msg = std_msgs::msg::Int8MultiArray();
            
            msg.data = {
                static_cast<int8_t>(mes[0]-'0'), 
                static_cast<int8_t>(mes[2]-'0'), 
                static_cast<int8_t>(mes[4]-'0'), 
                static_cast<int8_t>(mes[6]-'0'), 
                static_cast<int8_t>(mes[8]-'0')
            };

            m_returnToOriginStatusPublisher->publish(msg);
        }

        // Function to hangle motor status
        //  0 : Motor power off status
        //  1 : Motor power on status
        //  2 : Motor power on status + all servo on
        void handle_motor_status(const std::string& mes){
            auto msg = std_msgs::msg::Int8();
            
            msg.data = static_cast<int8_t>(mes[0]-'0');

            m_motorStatusPublisher->publish(msg);
        }

        // Function to hangle servo status
        // all 1,2,3,4,5,6 but we ignore 5 and 6
        void handle_servo_status(const std::string& mes){
            auto msg = std_msgs::msg::Int8MultiArray();
            
            msg.data = {
                static_cast<int8_t>(mes[0]-'0'), 
                static_cast<int8_t>(mes[2]-'0'), 
                static_cast<int8_t>(mes[4]-'0'), 
                static_cast<int8_t>(mes[6]-'0'), 
                static_cast<int8_t>(mes[8]-'0')
            };

            m_servoStatusPublisher->publish(msg);
        }

        // Function to hangle mode status
        //  0 : MANUAL MODE --> FBX ONLY
        //  1 : AUTO MODE --> Control source : Programming Box
        //  2 : AUTO MODE --> Control source release
        // -1 : RESTRICTED MODE 
        void handle_mode_status(const std::string& mes){
            auto msg = std_msgs::msg::Int8();
            
            msg.data = static_cast<int8_t>(std::stoi(mes));

            m_modeStatusPublisher->publish(msg);
        }

        // Function to check status of MoveTraj task
        // to know if the robot is busy
        // m,n,f,p
        // m : Execution programm number --> 1 to 100  [NOT USED]
        // n : Task execution line number --> 1 to 9999 [NOT USED]
        // f : task status --> R:RUN / U:SUSPEND / S:STOP / W:WAIT
        // p : Priority level --> 17 to 47 [NOT USED]
        void handle_task1_status(const std::string& mes){
            size_t first = mes.find(',');
            size_t second = mes.find(',', first + 1);


            if(mes[second+1]=='S' && m_busy) {
                m_busy = false;
                processNextMoveCommand();
            }
            else if(mes[second+1]=='R' && !m_busy) m_busy = true;
        }

};

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ControllerCom>());
    rclcpp::shutdown();

    return 0;
}