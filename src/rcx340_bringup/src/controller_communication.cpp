#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "yk400xe_interfaces/srv/movep_xy.hpp"
#include "yk400xe_interfaces/srv/movep_joints.hpp"

#include "rcx340_bringup/telnet.hpp"
#include "rcx340_bringup/errors.hpp"

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
    public:
        ControllerCom() : 
            Node("controller_communication_node"){
            
            m_handlerMap = {
                {1, [this](const std::string& s){handle_joints_state(s);}},
                {2, [this](const std::string& s){handle_XY_state(s);}}
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
                    -1707,        250000,           0.0,       -0.150,
                    -245760,      245760,    -6.2831853,    6.2831853 
                } // TODO Make symetrical values for 0 to be 0? TODO
            );

            this->get_parameter("controller_ip",m_ip);
            this->get_parameter("controller_port",m_port);
            this->get_parameter("joints_names",m_jointsNames);

            std::vector<double> flat;
            this->get_parameter("joints_ranges", flat);

            m_jointsRanges.clear();
            m_jointsRanges.reserve(flat.size() / 4);

            for (size_t i = 0; i < flat.size(); i += 4) {
            m_jointsRanges.push_back({
                flat[i + 0],
                flat[i + 1],
                flat[i + 2],
                flat[i + 3]
            });
            }

            m_jointStatesPublisher =
                this->create_publisher<sensor_msgs::msg::JointState>(
                "/joint_states", 10);

            m_xyStatePublisher =
                this->create_publisher<geometry_msgs::msg::Pose>(
                "/xy_states", 10);
            
            // Service the send command in cartesian coordinates
            m_movepxyService = this->create_service<yk400xe_interfaces::srv::MovepXY>(
                "/command/moveP_xy",
                std::bind(  &ControllerCom::handle_movepxy_service,
                            this,
                            std::placeholders::_1,
                            std::placeholders::_2
                        )
            );

            // Service the send command in joints values
            m_movepJointsService = this->create_service<yk400xe_interfaces::srv::MovepJoints>(
                "/command/moveP_joints",
                std::bind(  &ControllerCom::handle_movep_joints_service,
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

            // Object to deal handle Telnet communication with the controller
            m_telnet = new TelnetCommunication(m_ip, m_port,
                [this](const std::string& msg) {
                    this->process_message(msg);
                }
            );

            m_telnet->start();

            RCLCPP_INFO(this->get_logger(), "Communication node started");
        }

        ~ControllerCom(){
            m_telnet->stop();
        }

    private:

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_jointStatesPublisher;
        rclcpp::TimerBase::SharedPtr m_jointStateTimer;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_xyStatePublisher; 

        std::string m_ip;
        int m_port;

        TelnetCommunication* m_telnet;
        
        std::vector<std::string> m_jointsNames;
        std::vector<std::vector<double>> m_jointsRanges;

        std::deque<int> m_pendingCommands;

        std::unordered_map<int, std::function<void(const std::string&)>> m_handlerMap;

        rclcpp::Service<yk400xe_interfaces::srv::MovepXY>::SharedPtr m_movepxyService;
        rclcpp::Service<yk400xe_interfaces::srv::MovepJoints>::SharedPtr m_movepJointsService;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_servoService;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_alarmResetService;

        // Reset alarm
        void handle_alarm_reset_service(
            const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
            std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/){
            m_telnet->send_command("@ALMRST");
        }

        // Set the servos on/off
        void handle_set_servo_service(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> /*response*/){

            // Maybe add a check to see if servos are already on/off
            if(request->data){
                RCLCPP_INFO(
                    this->get_logger(),
                    "Starting Servos"
                );
                m_telnet->send_command("@SERVO ON");
            }
            else{
                RCLCPP_INFO(
                    this->get_logger(),
                    "Stoping Servos"
                );
                m_telnet->send_command("@SERVO OFF");
            }
        }

        // Send XY command to controller
        void handle_movepxy_service(
            const std::shared_ptr<yk400xe_interfaces::srv::MovepXY::Request> request,
            std::shared_ptr<yk400xe_interfaces::srv::MovepXY::Response> /*response*/){
                
            std::vector<std::string> cmd_msgs;
            float x,y,z,thz,s;

            x = request->posexyz.position.x * 1000;
            y = request->posexyz.position.y * 1000;
            z = request->posexyz.position.z * 1000;
            thz = request->posexyz.orientation.z;
            s = request->speed;

            RCLCPP_INFO(
                this->get_logger(),
                "Received request: x=%.3f y=%.3f z=%.3f thz=%.3f speed=%.1f",
                x, y, z, thz, s
            );

            std::string write_msg = "@WRITE P100";
            std::string point_msg = "P100= "; 
            std::string move_msg = "@MOVE P,P100,S=" ;
            
            point_msg += std::to_string(x) + " ";
            point_msg += std::to_string(y) + " ";
            point_msg += std::to_string(z) + " ";
            point_msg += std::to_string(thz)+ " 0.0 0.0";

            move_msg += std::to_string(s);

            cmd_msgs.push_back(write_msg);
            cmd_msgs.push_back(point_msg);
            cmd_msgs.push_back(move_msg);

            m_telnet->send_command(cmd_msgs);
        }

        // Send joint command to controller
        void handle_movep_joints_service(
            const std::shared_ptr<yk400xe_interfaces::srv::MovepJoints::Request> request,
            std::shared_ptr<yk400xe_interfaces::srv::MovepJoints::Response> /*response*/){
            
            std::vector<std::string> cmd_msgs;
            int s;
            
            std::vector<double> joint_states;
            std::vector<int> joint_states_remaped;
            joint_states.reserve(4);
            joint_states_remaped.reserve(4);

            joint_states.push_back(request->position[0]);
            joint_states.push_back(request->position[1]);
            joint_states.push_back(request->position[2]);
            joint_states.push_back(request->position[3]);

            joint_states_remaped = remap_to_joint_state(joint_states);
            s = request->speed;
            
            RCLCPP_INFO(
                this->get_logger(),
                "Received request: j1=%d j2=%d j3=%d j4=%d speed=%d",
                joint_states_remaped[0], joint_states_remaped[1], joint_states_remaped[2], joint_states_remaped[3], s
            );

            std::string write_msg = "@WRITE P100";
            std::string point_msg = "P100= "; 
            std::string move_msg = "@MOVE P,P100,S=" ;
            
            point_msg += std::to_string(joint_states_remaped[0]) + " ";
            point_msg += std::to_string(joint_states_remaped[1]) + " ";
            point_msg += std::to_string(-joint_states_remaped[2]) + " ";
            point_msg += std::to_string(joint_states_remaped[3])+ " 0 0";

            move_msg += std::to_string(s);

            cmd_msgs.push_back(write_msg);
            cmd_msgs.push_back(point_msg);
            cmd_msgs.push_back(move_msg);

            m_telnet->send_command(cmd_msgs);
        }

        // Send command to know joints States and end effectors
        void timer_callback(){
            std::string cmdJoints = "@?WHERE";
            std::string cmdXY = "@?WHRXY";
            int cmdIDJoints = 1;
            int cmdIDXY = 2;

            m_pendingCommands.push_back(cmdIDJoints);
            m_telnet->send_command(cmdJoints);

            m_pendingCommands.push_back(cmdIDXY);
            m_telnet->send_command(cmdXY);
        }

        // Remap joints state to cartesian coordinates 
        std::vector<double> remap_from_joint_state(std::vector<int>& pos){
            std::vector<double> remap_pos;
            remap_pos.reserve(pos.size());

            for (size_t i = 0; i<pos.size(); i++){
                double a = (m_jointsRanges[i][2]-m_jointsRanges[i][3])
                          /(m_jointsRanges[i][0]-m_jointsRanges[i][1]);
                double remap_state = a * pos[i] 
                    + (m_jointsRanges[i][2]-m_jointsRanges[i][0]*a);
                remap_pos.push_back(remap_state);
            }

            return remap_pos;
        }

        // Remap cartesian coordinates to joints state
        std::vector<int> remap_to_joint_state(const std::vector<double>& pos){
            std::vector<int> remap_pos;
            remap_pos.reserve(pos.size());

            for (size_t i = 0; i < pos.size(); i++) {
                double a = (m_jointsRanges[i][2] - m_jointsRanges[i][3])
                          /(m_jointsRanges[i][0] - m_jointsRanges[i][1]);

                double b = m_jointsRanges[i][2] - m_jointsRanges[i][0] * a;
                double joint_state = (pos[i] - b) / a;

                remap_pos.push_back(static_cast<int>(joint_state));
            }

            return remap_pos;
        }

        void process_message(const std::string& msg){
            
            // Start or End of tasks, SETUP TODO
            if (msg=="OK" ||    // Command tracking --> setup TODO
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
                        "%s :  %s",
                        error_code.c_str(),
                        it->second.c_str()
                    );
                else
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Unknown error : %s",
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
            std::vector<int> pulse_positions;
            std::vector<double> ros_positions;

            size_t pos ;
            size_t start = 0;

            for (int i=0; i<4; i++){
                pos = mes.find(' ',start);
                std::string state = mes.substr(start, pos);

                double value;
                std::stringstream ss(state);
                ss >> value;
                pulse_positions.push_back(value);
                start = pos+1;
            }

            ros_positions = remap_from_joint_state(pulse_positions);

            std_msgs::msg::Header header;
            header.stamp = this->now();
            header.frame_id = "base_link"; 

            sensor_msgs::msg::JointState joint_state_msg;
            joint_state_msg.header=header;
            joint_state_msg.name=m_jointsNames;
            joint_state_msg.position=ros_positions;

            m_jointStatesPublisher->publish(joint_state_msg);
        }

        // Function to parse and publish XY states
        void handle_XY_state(const std::string& mes){
            std::vector<double> xyPositions;

            size_t pos ;
            size_t start = 0;

            for (int i=0; i<4; i++){
                pos = mes.find(' ',start);
                std::string state = mes.substr(start, pos);

                double value;
                std::stringstream ss(state);
                ss >> value;
                xyPositions.push_back(value);
                start = pos+1;
            }
            
            geometry_msgs::msg::Point xyPoint;
            xyPoint.x = xyPositions[0];
            xyPoint.y = xyPositions[1];
            xyPoint.z = xyPositions[2];

            geometry_msgs::msg::Quaternion xyQuaternion;
            xyQuaternion.z = xyPositions[3];

            geometry_msgs::msg::Pose xyMsg;
            xyMsg.position = xyPoint;
            xyMsg.orientation = xyQuaternion;

            m_xyStatePublisher->publish(xyMsg);
        }

};

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ControllerCom>());
    rclcpp::shutdown();

    return 0;
}