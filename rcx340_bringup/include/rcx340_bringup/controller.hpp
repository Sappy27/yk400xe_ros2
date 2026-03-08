#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "rcx340_bringup/telnet.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "yk400xe_interfaces/msg/move.hpp"

#include <string>
#include <functional>
#include <deque>
#include <deque>
#include <vector>


class Controller{

  private:
    TelnetCommunication* m_telnet;
    std::function<void(const std::string& msg)> m_msg_cb;
    std::vector<std::vector<double>> m_jointsRanges;

  public:
    Controller(std::string ip, int port, 
        std::vector<double> flat,
        std::function<void(const std::string& msg)> cb = nullptr)
        : m_msg_cb(std::move(cb))  {

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

      // Object to deal handle Telnet communication with the controller
      m_telnet = new TelnetCommunication(ip, port,
        [this](const std::string& msg) {
           this->m_msg_cb(msg);
        } 
      );

      m_telnet->start();
    }

    ~Controller(){
      m_telnet->stop();
    }
    
    // ==========================================
    // System Status
    // ==========================================

    void where(){
      std::string cmd = "@?WHERE";
      m_telnet->send_command(cmd);
    }

    void where_xy(){
      std::string cmd = "@?WHRXY";
      m_telnet->send_command(cmd);
    }

    void alarm_status(){
      std::string cmd = "@?EMG";
      m_telnet->send_command(cmd);
    }

    void mspeed_status(){
      std::string cmd = "@?MSPEED";
      m_telnet->send_command(cmd);
    }

    void return_to_origin_status(){
      std::string cmd = "@?ORIGIN";
      m_telnet->send_command(cmd);
    }

    void motor_status(){
      std::string cmd = "@?MOTOR";
      m_telnet->send_command(cmd);
    }

    void servo_status(){
      std::string cmd = "@?SERVO";
      m_telnet->send_command(cmd);
    }

    // ==========================================
    // Coordinate Control
    // ==========================================

    // CHANGE, HAND, SHIFT, LEFTY, RIGHTY

    void set_lefty_righty(int hand){
      std::string cmd;
      if(hand==1){
        cmd += "@RIGHTY";
      }
      else if(hand==2){
        cmd += "@LEFTY";
      }
      else{
        return;
      }

      m_telnet->send_command(cmd);
    }

    // ==========================================
    // Status Change
    // ==========================================

    // ACCEL, ARCH, ASPEED, AXWGHT, DECEL 
    // ORGORD, OUTPOS, SPEED, TOLE, WEIGHT

    void set_accel(int accell, int axis=0){
      std::string cmd = "@ACCEL ";
      if(axis>0){
        cmd+="(" + std::to_string(axis) + ")" + "=";
      }
      cmd+=std::to_string(accell);

      m_telnet->send_command(cmd);
    }

    void set_arch(int arch, int axis=0){
      std::string cmd = "@ARCH ";
      if(axis>0){
        cmd+="(" + std::to_string(axis) + ")" + "=";
      }
      cmd+=std::to_string(arch);

      m_telnet->send_command(cmd);
    }

    void set_aspeed(int aspeed){
      std::string cmd = "@ASPEED ";
      cmd+=std::to_string(aspeed);

      m_telnet->send_command(cmd);
    }

    void set_axwght(int axwght, int axis=0){
      std::string cmd = "@AXWGHT ";
      if(axis>0){
        cmd+="(" + std::to_string(axis) + ")" + "=";
      }
      cmd+=std::to_string(axwght);

      m_telnet->send_command(cmd);
    }

    void set_decel(int decel, int axis=0){
      std::string cmd = "@DECEL ";
      if(axis>0){
        cmd+="(" + std::to_string(axis) + ")" + "=";
      }
      cmd+=std::to_string(decel);

      m_telnet->send_command(cmd);
    } 

    void set_outpos(int outpos, int axis=0){
      std::string cmd = "@OUTPOS ";
      if(axis>0){
        cmd+="(" + std::to_string(axis) + ")" + "=";
      }
      cmd+=std::to_string(outpos);

      m_telnet->send_command(cmd);
    } 

    void set_mspeed(int mspeed){
      std::string cmd = "@MSPEED ";
      cmd+=std::to_string(mspeed);

      m_telnet->send_command(cmd);
    } 

    void teach_pulse(int n){
      std::string cmd = "@TEACH ";
      cmd+=std::to_string(n);

      m_telnet->send_command(cmd);
    }   

    void teach_xy(int n){
      std::string cmd = "@TEACHXY ";
      cmd+=std::to_string(n);

      m_telnet->send_command(cmd);
    }  

    void alarm_reset(){
      m_telnet->send_command("@ALMRST");
    }

    // ==========================================
    // Robot Movement 
    // ==========================================

    void servo_on(){
      m_telnet->send_command("@SERVO ON");
    }

    void servo_off(){
      m_telnet->send_command("@SERVO OFF");
    }

    void origin_return(){
      m_telnet->send_command("@ORGRTN");
    }

    // ==========================================
    // Robot Trajectory
    // ==========================================
    
    // Remap joints pulses to angles or length
    std::array<double,4> remap_from_pulse(std::array<int,4>& pos){
        std::array<double,4> remap_pos;

        for (size_t i = 0; i<pos.size(); i++){
            double a = (m_jointsRanges[i][2]-m_jointsRanges[i][3])
                      /(m_jointsRanges[i][0]-m_jointsRanges[i][1]);
            double remap_state = a * pos[i] 
                + (m_jointsRanges[i][2]-m_jointsRanges[i][0]*a);
            remap_pos[i]=static_cast<double>(remap_state);
        }

        return remap_pos;
    }

    // Remap joints angles or length to pulses
    std::array<int,4> remap_to_pulse(const std::array<double,4>& pos){
      std::array<int,4> remap_pos;

      for (size_t i = 0; i < pos.size(); i++) {
        double a = (m_jointsRanges[i][2] - m_jointsRanges[i][3])
                  /(m_jointsRanges[i][0] - m_jointsRanges[i][1]);

        double b = m_jointsRanges[i][2] - m_jointsRanges[i][0] * a;
        double joint_state = (pos[i] - b) / a;

        remap_pos[i]=static_cast<int>(joint_state);
      }

      return remap_pos;
    }

    double remap_j4_std_coord(double j4){
      double a = (m_jointsRanges[4][2] - m_jointsRanges[4][3])
                /(m_jointsRanges[4][0] - m_jointsRanges[4][1]);

      double b = m_jointsRanges[4][2] - m_jointsRanges[4][0] * a;
      double joint_state = (j4 - b) / a;

      return joint_state;
    }

    void move_trajectory(
        std::vector<yk400xe_interfaces::msg::Move> moveCmd){

      std::vector<std::string> cmd_msg;
      cmd_msg.push_back("@WRITE <TRAJ_PG>"); 
      cmd_msg.push_back("NAME=TRAJ_PG"); 
      cmd_msg.push_back("PGN=1"); 

      int i = 0;
      
      while(i<int(moveCmd.size())){
        auto& mi = moveCmd[i];
        
        if(mi.coords_type==0){  // Std Coordinates
          std::array<double,4> pose = mi.pose;

          std::string line = "P"+std::to_string(100+i)+"=";
          line += std::to_string(pose[0] * 1000) + " ";
          line += std::to_string(pose[1] * 1000) + " ";
          line += std::to_string(pose[2] * 1000) + " ";
          line += std::to_string(remap_j4_std_coord(pose[3])) + " 0.0 0.0";
          cmd_msg.push_back(line);

          if(mi.move_type==3){
            std::array<double,4> pose2 = mi.pose2;
            line.erase();
            i++;
            line = "P"+std::to_string(100+i)+"=";
            line += std::to_string(pose2[0] * 1000) + " ";
            line += std::to_string(pose2[1] * 1000) + " ";
            line += std::to_string(pose2[2] * 1000) + " ";
            line += std::to_string(remap_j4_std_coord(pose[3])) + " 0.0 0.0";
            cmd_msg.push_back(line);
          }
          
          i++;
        }
        else {  // Pulses
          while(i<int(moveCmd.size())){
            std::array<int,4> pose_pulse = remap_to_pulse(mi.pose);

            std::string line = "P"+std::to_string(100+i)+"=";
            line += std::to_string(pose_pulse[0]) + " ";
            line += std::to_string(pose_pulse[1]) + " ";
            line += std::to_string(pose_pulse[2]) + " ";
            line += std::to_string(pose_pulse[3]) + " 0 0";
            cmd_msg.push_back(line);

            if(mi.move_type==3){
              std::array<int,4> pose2_pulse = remap_to_pulse(mi.pose2);

              line.erase();
              i++;
              line = "P"+std::to_string(100+i)+"=";
              line += std::to_string(pose2_pulse[0]) + " ";
              line += std::to_string(pose2_pulse[1]) + " ";
              line += std::to_string(pose2_pulse[2]) + " ";
              line += std::to_string(pose2_pulse[3]) + " 0 0";
              cmd_msg.push_back(line);
            }
            
            i++;
          }
        }
      }

      i=0;
      while(i<int(moveCmd.size())){
        std::string line = "MOVE";
        auto& mi = moveCmd[i];
        switch(mi.move_type){
          default:
          case 1: 
            line+=" P,P" + std::to_string(100+i);
            break;
          case 2: 
            line+=" L,P" + std::to_string(100+i);
            break;
          case 3: 
            line+=" C,P" + std::to_string(100+i) + "P" + std::to_string(100+i+1);
            i++;
            break;
          case 4: 
            line+="I,P" + std::to_string(100+i);
            break;
        }

        if(mi.speed>0){
          switch(mi.move_type){
            default:
            case 1: 
            case 4: 
              line+=",S=" + std::to_string(mi.speed);
              break;
            case 2: 
            case 3: 
              line+=",VEL=" + std::to_string(mi.speed);
              break;
          }
        }

        if(mi.do_arch && mi.move_type==1){
          line+=",A3=" + std::to_string(mi.arch[0]);
          line+="{" + std::to_string(mi.arch[1]) + 
                      std::to_string(mi.arch[2]) + "}";
        }

        if(mi.cont){
          line+=",CONT";
        }

        i++;
        cmd_msg.push_back(line);
      }

      cmd_msg.push_back("");
      cmd_msg.push_back("@LOAD <TRAJ_PG>,T1");
      cmd_msg.push_back("@RUN T1");

      m_telnet->send_command(cmd_msg);
    }

};

#endif //CONTROLLER_HPP