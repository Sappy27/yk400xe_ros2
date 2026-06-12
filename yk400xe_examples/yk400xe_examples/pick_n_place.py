#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from yk400xe_interfaces.srv import MoveTrajectory
from yk400xe_interfaces.msg import Move
from geometry_msgs.msg import PoseStamped

import time
import threading
import numpy as np

class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_n_place_node")

        self.lock = threading.Lock()

        self.ef_ = []

        self.home_pose_ = [0.0, 0.0, 0.0, 0.0]
        self.pick_pose_ = [0.0, 0.0, 0.0, 0.0]
        self.place_pose_ = [0.0, 0.0, 0.0, 0.0]


        self.ef_sub_ = self.create_subscription(
            PoseStamped,
            "/ee_state",
            self.ef_state_cb,
            10)


        self.move_client_ = self.create_client(
            MoveTrajectory,
            "/command/move",
        )
        while not self.move_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Move commands service not available, trying again")

        while len(self.ef_) == 0:
            rclpy.spin_once(self)

        self.input_thread = threading.Thread(target=self.run, daemon=True)
        self.input_thread.start()


    def ef_state_cb(self,msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        self.ef_ = [float(x), float(y), float(z), 0.0]


    def input_loop(self):
        while True:
            input("Press any key")
            with self.lock:
                print(f"ef {self.ef_}")

    def set_pose(self):
        input("Place the robot in home position and press ENTER")
        with self.lock:
            self.home_pose_ = self.ef_.copy()

        input("Place the robot in pick position and press ENTER")
        with self.lock:
            self.pick_pose_ = self.ef_.copy()

        input("Place the robot in place position and press ENTER")
        with self.lock:
            self.place_pose_ = self.ef_.copy()


    def is_at(self,pose,eps=0.01):
        with self.lock:
            d = np.linalg.norm(np.array(pose[:3]) - np.array(self.ef_[:3]))
        return(d<eps)


    def go_to(self,pose,arch=True):
        move_request = MoveTrajectory.Request()

        print(pose)
        move_msg = Move()
        move_msg.pose = pose
        move_msg.do_arch = arch
        move_msg.arch = [0.0,0.0,0.0]
        move_msg.speed = 10
        move_msg.coords_type = 0  # Std Coordinates
        move_msg.move_type = 1  # P
        move_msg.wait_arm = False

        move_request.move_cmds = [move_msg]
        self.future = self.move_client_.call_async(move_request)
    
    
    def run(self):
        self.set_pose()

        self.get_logger().info(f"Home {self.home_pose_}")
        self.get_logger().info(f"Pick {self.pick_pose_}")
        self.get_logger().info(f"Place {self.place_pose_}")

        while(True):
            input("press ENTER to start")
            print(self.home_pose_)
            print(self.pick_pose_)
            print(self.place_pose_)

            self.go_to(self.home_pose_,arch=False)
            while(not self.is_at(self.home_pose_)):
                time.sleep(0.01)

            time.sleep(0.1)
            self.go_to(self.pick_pose_)
            while(not self.is_at(self.pick_pose_)):
                time.sleep(0.01)         

            # Add your own gripping method (vacuum, finger gripper, etc.) here

            time.sleep(0.1)
            self.go_to(self.place_pose_)
            while(not self.is_at(self.place_pose_)):
                time.sleep(0.01)
            
            time.sleep(0.2)
            self.go_to(self.home_pose_,arch=False)
            while(not self.is_at(self.home_pose_)):
                time.sleep(0.01)
            time.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    node = PickAndPlace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == "__main__":
    main()