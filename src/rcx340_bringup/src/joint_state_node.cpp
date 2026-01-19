#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"

#include "rcx340_bringup/telnet.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <math.h>

#include <iostream>

class JointStatePublisher : public rclcpp::Node{
    public:
        JointStatePublisher() : 
            Node("joint_state_node"),
            telnet(ip_, port_,
                [this](const std::string& msg) {
                    this->publish_joint_state(msg);
                }
            ){
            joint_states_publisher_=this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

            joint_state_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&JointStatePublisher::timer_callback, this)
            );

            telnet.start();
        }

        ~JointStatePublisher(){
            telnet.stop();
        }

    private:

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
        rclcpp::TimerBase::SharedPtr joint_state_timer_;

        const std::string ip_ = "192.168.0.2";
        unsigned int port_ = 23;

        TelnetCommunication telnet;
        
        const std::vector<std::string> joints_names_ = {"J1","J2","J3","J4"} ;
        const std::vector<std::array<double,4>> limits = {
            {-385506,380555,-132*M_PI/180,132*M_PI/180},
            {-437972,439198,-150*M_PI/180,150*M_PI/180},
            {-1707,256000,0,-0.150},
            {-245760,245760,-2*M_PI,2*M_PI}};

        void timer_callback(){
            std::string cmd = "@?WHERE";
            telnet.send_command(cmd);
        }

        std::vector<double> remap_joint_states(std::vector<int> pos){
            std::vector<double> remap_pos;
            for (size_t i = 0; i<pos.size(); i++){
                double a = (limits[i][2]-limits[i][3])/(limits[i][0]-limits[i][1]);
                double remap_state = a * pos[i] + (limits[i][2]-limits[i][0]*a);
                remap_pos.push_back(remap_state);
            }

            return remap_pos;
        }

        void publish_joint_state(const std::string& mes){
            
            if(mes=="OK" || mes=="END" || mes=="BEGIN" || mes.empty()) return;

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

            ros_positions = remap_joint_states(pulse_positions);

            std_msgs::msg::Header header;
            header.stamp = this->now();
            header.frame_id = "base_link"; 

            sensor_msgs::msg::JointState joint_state_msg;
            joint_state_msg.header=header;
            joint_state_msg.name=joints_names_;
            joint_state_msg.position=ros_positions;

            joint_states_publisher_->publish(joint_state_msg);
        }
        
};

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<JointStatePublisher>());
    rclcpp::shutdown();

    return 0;
}