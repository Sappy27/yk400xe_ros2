#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from yk400xe_interfaces.srv import MoveTrajectory
from yk400xe_interfaces.msg import Move

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from sensor_msgs_py import point_cloud2


class DrawYamahaRos(Node):
    def __init__(self):
        super().__init__("draw_yamaha_ros_node")

        self.subscriber_ = self.create_subscription(PoseStamped,"/ee_state",self.ef_state_cb,10)

        self.publisher_ = self.create_publisher(PointCloud2,"/draw_pc",10)

        self.x0, self.y0, self.z0 = None, None, None
        self.x, self.y, self.z = None, None, None

        self.points = []
        
        self.ini = False

        self.timer = self.create_timer(0.01, self.run)

        self.client_ = self.create_client(MoveTrajectory,"/command/move")
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, trying again")

        self.safety_check()
       
        self.get_logger().info("Node initialization done")
        
        self.run()

    def safety_check(self):
        inp = input("\033[38;5;202mAre you sure the inputs coordinates are correct ?\n\033[0mEnter for yes / Ctrl+C for no\n")
        if (inp == "admin") : return

        inp = input("\033[38;5;202mIs there any obstacle that could collide with the robot ?\n\033[0mEnter for yes / Ctrl+C for no\n")
        if (inp == "admin") : return

        inp = input("\033[38;5;202mAre you ready to trigger emergency stop if needed ?\n\033[0mEnter for yes / Ctrl+C for no\n")
        if (inp == "admin") : return
        return
    
    def ef_state_cb(self, msg:PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        self.points.append([self.x,self.y,self.z])
        return

    def send_command(self, cmd):
        request = MoveTrajectory.Request()
        request.move_cmds = cmd
        self.client_.call_async(request)
    
    def draw_H(self,x0,y0,z0):
        x1,y1,z1 = x0,y0,z0-0.01
        x2,y2,z2 = x1,y1-0.05,z1
        x3,y3,z3 = x2,y2+0.025,z2
        x4,y4,z4 = x3+0.025,y3,z3
        x5,y5,z5 = x4,y4+0.025,z4
        x6,y6,z6 = x5,y5-0.050,z5

        Move0=Move()
        Move0.move_type=1
        Move0.coords_type=0
        Move0.pose=[x0,y0,z0,0.0]
        Move0.speed=99
        Move0.wait_arm=False

        Move1=Move()
        Move1.move_type=1
        Move1.coords_type=0
        Move1.pose=[x1,y1,z1,0.0]
        Move1.speed=99
        Move1.wait_arm=False

        Move2=Move()
        Move2.move_type=2
        Move2.coords_type=0
        Move2.pose=[x2,y2,z2,0.0]
        Move2.speed=99
        Move2.wait_arm=False

        Move3=Move()
        Move3.move_type=1
        Move3.coords_type=0
        Move3.pose=[x3,y3,z3,0.0]
        Move3.speed=100
        Move3.do_arch = True
        Move3.arch = [z0,0.0,0.0] 
        Move3.wait_arm=False

        Move4=Move()
        Move4.move_type=2
        Move4.coords_type=0
        Move4.pose=[x4,y4,z4,0.0]
        Move4.speed=99
        Move4.wait_arm=False

        Move5=Move()
        Move5.move_type=1
        Move5.coords_type=0
        Move5.pose=[x5,y5,z5,0.0]
        Move5.speed=100
        Move5.do_arch = True
        Move5.arch = [z0,0.0,0.0] 
        Move5.wait_arm=False

        Move6=Move()
        Move6.move_type=2
        Move6.coords_type=0
        Move6.pose=[x6,y6,z6,0.0]
        Move6.speed=99
        Move6.wait_arm=False

        move_cmds = [
            Move0,
            Move1,
            Move2,
            Move3,
            Move4,
            Move5,
            Move6
        ]

        return move_cmds
    
    def draw_A(self,x0,y0,z0):
        x1,y1,z1 = x0,y0,z0-0.01
        x2,y2,z2 = x1+0.0125,y1+0.05,z1
        x3,y3,z3 = x2+0.0125,y2-0.05,z2
        x4,y4,z4 = x1+0.003,y1+0.01,z3
        x5,y5,z5 = x3-0.003,y3+0.01,z4
        

        Move0=Move()
        Move0.move_type=1
        Move0.coords_type=0
        Move0.pose=[x0,y0,z0,0.0]
        Move0.speed=100
        Move0.wait_arm=False
        Move0.do_arch = True
        Move0.arch = [z0,0.0,0.0] 

        Move1=Move()
        Move1.move_type=1
        Move1.coords_type=0
        Move1.pose=[x1,y1,z1,0.0]
        Move1.speed=100
        Move1.wait_arm=False

        Move2=Move()
        Move2.move_type=2
        Move2.coords_type=0
        Move2.pose=[x2,y2,z2,0.0]
        Move2.speed=99
        Move2.wait_arm=False

        Move3=Move()
        Move3.move_type=2
        Move3.coords_type=0
        Move3.pose=[x3,y3,z3,0.0]
        Move3.speed=99
        Move3.wait_arm=False

        Move4=Move()
        Move4.move_type=1
        Move4.coords_type=0
        Move4.pose=[x4,y4,z4,0.0]
        Move4.speed=100
        Move4.wait_arm=False
        Move4.do_arch = True
        Move4.arch = [z0,0.0,0.0] 

        Move5=Move()
        Move5.move_type=2
        Move5.coords_type=0
        Move5.pose=[x5,y5,z5,0.0]
        Move5.speed=99
        Move5.wait_arm=False

        move_cmds = [
            Move0,
            Move1,
            Move2,
            Move3,
            Move4,
            Move5
        ]

        return move_cmds
    

    def draw_Y(self,x0,y0,z0):
        x1,y1,z1 = x0,y0,z0-0.01
        x2,y2,z2 = x1+0.0125,y1-0.025,z1
        x3,y3,z3 = x2+0.0125,y2+0.025,z2
        x4,y4,z4 = x2,y2,z2
        x5,y5,z5 = x4,y4-0.025,z4
        

        Move0=Move()
        Move0.move_type=1
        Move0.coords_type=0
        Move0.pose=[x0,y0,z0,0.0]
        Move0.speed=100
        Move0.wait_arm=False
        Move0.do_arch = True
        Move0.arch = [z0,0.0,0.0] 

        Move1=Move()
        Move1.move_type=1
        Move1.coords_type=0
        Move1.pose=[x1,y1,z1,0.0]
        Move1.speed=100
        Move1.wait_arm=False

        Move2=Move()
        Move2.move_type=2
        Move2.coords_type=0
        Move2.pose=[x2,y2,z2,0.0]
        Move2.speed=99
        Move2.wait_arm=False

        Move3=Move()
        Move3.move_type=2
        Move3.coords_type=0
        Move3.pose=[x3,y3,z3,0.0]
        Move3.speed=99
        Move3.wait_arm=False

        Move4=Move()
        Move4.move_type=1
        Move4.coords_type=0
        Move4.pose=[x4,y4,z4,0.0]
        Move4.speed=100
        Move4.wait_arm=False
        Move4.do_arch = True
        Move4.arch = [z0,0.0,0.0] 

        Move5=Move()
        Move5.move_type=2
        Move5.coords_type=0
        Move5.pose=[x5,y5,z5,0.0]
        Move5.speed=99
        Move5.wait_arm=False

        move_cmds = [
            Move0,
            Move1,
            Move2,
            Move3,
            Move4,
            Move5
        ]

        return move_cmds
    
    def draw_M(self,x0,y0,z0):
        x1,y1,z1 = x0,y0,z0-0.01
        x2,y2,z2 = x1,y1+0.05,z1
        x3,y3,z3 = x2+0.0125,y2-0.025,z2
        x4,y4,z4 = x3+0.0125,y3+0.025,z2
        x5,y5,z5 = x4,y4-0.05,z4
        

        Move0=Move()
        Move0.move_type=1
        Move0.coords_type=0
        Move0.pose=[x0,y0,z0,0.0]
        Move0.speed=100
        Move0.wait_arm=False
        Move0.do_arch = True
        Move0.arch = [z0,0.0,0.0] 

        Move1=Move()
        Move1.move_type=2
        Move1.coords_type=0
        Move1.pose=[x1,y1,z1,0.0]
        Move1.speed=99
        Move1.wait_arm=False

        Move2=Move()
        Move2.move_type=2
        Move2.coords_type=0
        Move2.pose=[x2,y2,z2,0.0]
        Move2.speed=99
        Move2.wait_arm=False

        Move3=Move()
        Move3.move_type=2
        Move3.coords_type=0
        Move3.pose=[x3,y3,z3,0.0]
        Move3.speed=99
        Move3.wait_arm=False

        Move4=Move()
        Move4.move_type=2
        Move4.coords_type=0
        Move4.pose=[x4,y4,z4,0.0]
        Move4.speed=99
        Move4.wait_arm=False

        Move5=Move()
        Move5.move_type=2
        Move5.coords_type=0
        Move5.pose=[x5,y5,z5,0.0]
        Move5.speed=99
        Move5.wait_arm=False

        move_cmds = [
            Move0,
            Move1,
            Move2,
            Move3,
            Move4,
            Move5
        ]

        return move_cmds
    

    def draw_R(self,x0,y0,z0):
        x1,y1,z1 = x0,y0,z0-0.01
        x2,y2,z2 = x1,y1+0.05,z1
        x3,y3,z3 = x2+0.0125,y2,z2
        x4_1,y4_1,z4_1 = x3+0.0125,y3-0.0125,z2
        x4_2,y4_2,z4_2 = x3,y3-0.025,z4_1
        x5,y5,z5 = x4_2-0.0125,y4_2,z1
        x6,y6,z6 = x1+0.025,y1,z1

        

        Move0=Move()
        Move0.move_type=1
        Move0.coords_type=0
        Move0.pose=[x0,y0,z0,0.0]
        Move0.speed=100
        Move0.wait_arm=False
        Move0.do_arch = True
        Move0.arch = [z0,0.0,0.0] 

        Move1=Move()
        Move1.move_type=1
        Move1.coords_type=0
        Move1.pose=[x1,y1,z1,0.0]
        Move1.speed=100
        Move1.wait_arm=False

        Move2=Move()
        Move2.move_type=2
        Move2.coords_type=0
        Move2.pose=[x2,y2,z2,0.0]
        Move2.speed=99
        Move2.wait_arm=False

        Move3=Move()
        Move3.move_type=2
        Move3.coords_type=0
        Move3.pose=[x3,y3,z3,0.0]
        Move3.speed=99
        Move3.wait_arm=False

        Move4=Move()
        Move4.move_type=3
        Move4.coords_type=0
        Move4.pose=[x4_1,y4_1,z4_1,0.0]
        Move4.pose2=[x4_2,y4_2,z4_2,0.0]
        Move4.speed=99
        Move4.wait_arm=False

        Move5=Move()
        Move5.move_type=2
        Move5.coords_type=0
        Move5.pose=[x5,y5,z5,0.0]
        Move5.speed=99
        Move5.wait_arm=False

        Move6=Move()
        Move6.move_type=1
        Move6.coords_type=0
        Move6.pose=[x4_2,y4_2,z4_2,0.0]
        Move6.speed=100
        Move6.wait_arm=False
        Move6.do_arch = True
        Move6.arch = [z0,0.0,0.0] 

        Move7=Move()
        Move7.move_type=2
        Move7.coords_type=0
        Move7.pose=[x6,y6,z6,0.0]
        Move7.speed=99
        Move7.wait_arm=False

        move_cmds = [
            Move0,
            Move1,
            Move2,
            Move3,
            Move4,
            Move5,
            Move6,
            Move7
        ]

        return move_cmds

    def draw_S(self,x0,y0,z0):
        x1,y1,z1 = x0,y0,z0-0.01
        x2,y2,z2 = x1-0.0125,y1,z1
        x3_1,y3_1 = x2-0.0125,y2-0.0125
        x3_2,y3_2 = x2,y2-0.025
        x4_1,y4_1 = x3_2+0.0125,y3_2-0.0125
        x4_2,y4_2 = x3_2,y3_2-0.025
        x5,y5,z5 = x4_2-0.0125,y4_2,z2

        Move0=Move()
        Move0.move_type=1
        Move0.coords_type=0
        Move0.pose=[x0,y0,z0,0.0]
        Move0.speed=100
        Move0.wait_arm=False
        Move0.do_arch = True
        Move0.arch = [z0,0.0,0.0] 

        Move1=Move()
        Move1.move_type=1
        Move1.coords_type=0
        Move1.pose=[x1,y1,z1,0.0]
        Move1.speed=100
        Move1.wait_arm=False

        Move2=Move()
        Move2.move_type=2
        Move2.coords_type=0
        Move2.pose=[x2,y2,z2,0.0]
        Move2.speed=99
        Move2.wait_arm=False

        Move3=Move()
        Move3.move_type=3
        Move3.coords_type=0
        Move3.pose=[x3_1,y3_1,z2,0.0]
        Move3.pose2=[x3_2,y3_2,z2,0.0]
        Move3.speed=99
        Move3.wait_arm=False

        Move4=Move()
        Move4.move_type=3
        Move4.coords_type=0
        Move4.pose=[x4_1,y4_1,z2,0.0]
        Move4.pose2=[x4_2,y4_2,z2,0.0]
        Move4.speed=99
        Move4.wait_arm=False

        Move5=Move()
        Move5.move_type=2
        Move5.coords_type=0
        Move5.pose=[x5,y5,z5,0.0]
        Move5.speed=99
        Move5.wait_arm=False

        move_cmds = [
            Move0,
            Move1,
            Move2,
            Move3,
            Move4,
            Move5
        ]

        return move_cmds
    
    def draw_O(self,x0,y0,z0):
        x1,y1,z1 = x0,y0,z0-0.01
        x2_1,y2_1 = x1+0.0125,y1+0.0125
        x2_2,y2_2 = x1+0.025,y1
        x3,y3 = x2_2,y2_2-0.025
        x4_1,y4_1 = x3-0.0125,y3-0.0125
        x4_2,y4_2 = x3-0.025,y3

        Move0=Move()
        Move0.move_type=1
        Move0.coords_type=0
        Move0.pose=[x0,y0,z0,0.0]
        Move0.speed=100
        Move0.wait_arm=False
        Move0.do_arch = True
        Move0.arch = [z0,0.0,0.0] 

        Move1=Move()
        Move1.move_type=1
        Move1.coords_type=0
        Move1.pose=[x1,y1,z1,0.0]
        Move1.speed=100
        Move1.wait_arm=False

        Move2=Move()
        Move2.move_type=3
        Move2.coords_type=0
        Move2.pose=[x2_1,y2_1,z1,0.0]
        Move2.pose2=[x2_2,y2_2,z1,0.0]
        Move2.speed=99
        Move2.wait_arm=False

        Move3=Move()
        Move3.move_type=2
        Move3.coords_type=0
        Move3.pose=[x3,y3,z1,0.0]
        Move3.speed=99
        Move3.wait_arm=False

        Move4=Move()
        Move4.move_type=3
        Move4.coords_type=0
        Move4.pose=[x4_1,y4_1,z1,0.0]
        Move4.pose2=[x4_2,y4_2,z1,0.0]
        Move4.speed=99
        Move4.wait_arm=False

        Move5=Move()
        Move5.move_type=2
        Move5.coords_type=0
        Move5.pose=[x1,y1,z1,0.0]
        Move5.speed=99
        Move5.wait_arm=False

        move_cmds = [
            Move0,
            Move1,
            Move2,
            Move3,
            Move4,
            Move5
        ]

        return move_cmds

    def move_to(self,x,y,z):
        Move0=Move()
        Move0.move_type=1
        Move0.coords_type=0
        Move0.pose=[x,y,z,0.0]
        Move0.speed=100
        Move0.wait_arm=True
        Move0.do_arch = False
        Move0.arch = [z,0.0,0.0] 
        return [Move0]

    def run(self):
        if(self.x!=None and self.y!=None and self.z!=None):
            if(not self.ini):
                input("Press Enter to send command")
                cmd = (
                    self.draw_Y(-0.115,0.28,-0.085) + 
                    self.draw_A(-0.08,0.23,-0.085)  + 
                    self.draw_M(-0.04,0.23,-0.085) + 
                    self.draw_A(-0.0,0.23,-0.085)  + 
                    self.draw_H(0.04,0.28,-0.085)   +
                    self.draw_A(0.08,0.23,-0.085)   + 
                    self.draw_R(-0.07,0.16,-0.085)  + 
                    self.draw_O(-0.02,0.2,-0.085)   +
                    self.draw_S(0.05,0.21,-0.085)   + 
                    self.move_to(0.0,0.30,0.0)
                ) 
                

                self.send_command(cmd)
                self.ini = True

            header=Header()
            header.frame_id="std_coords_link"
            header.stamp=self.get_clock().now().to_msg()

            cloud_msg = point_cloud2.create_cloud_xyz32(header, self.points)

            self.publisher_.publish(cloud_msg)


def main():
    rclpy.init()
    node = DrawYamahaRos()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()