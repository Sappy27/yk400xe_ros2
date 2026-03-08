#!/usr/bin/env python3

import numpy as np
import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from yk400xe_interfaces.srv import MoveTrajectory
from yk400xe_interfaces.msg import Move

from PyQt6.QtCore import Qt,QSize

from PyQt6.QtWidgets import (QWidget,QSlider,QPushButton, QMainWindow,
                             QGridLayout,QLabel,QLayout,QApplication,
                             QHBoxLayout,QVBoxLayout)

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.backend_bases import MouseEvent
from matplotlib.path import Path

#==============

X0 = 0.0
Y0 = -0.0

L1 = 0.225 # m
L2 = 0.175 # m

TH1MIN = -132 * np.pi/180 # rad
TH1MAX = 132 * np.pi/180 # rad

TH2MIN = -150 * np.pi/180 # rad
TH2MAX = 150 * np.pi/180 # rad

ZMIN = 0 # mm
ZMAX = 150 # mm

THZMIN = -360 # deg
THZMAX = 360 # deg

# =============

class CommandNode(Node):
    def __init__(self):
        super().__init__("command_gui_xy_node")

        self.client = self.create_client(MoveTrajectory,"/command/move")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, trying again")
        
    def send_command(
            self,
            x : float, 
            y : float, 
            z : float, 
            thz : float,
            s : int 
        ) -> None:

        request = MoveTrajectory.Request()
        move = Move()
        move.pose = [
                    float(x), 
                    float(y), 
                    float(z), 
                    float(thz)*np.pi/180
                ]
        move.coords_type = 0 # std coords
        move.move_type = 1 # P
        move.speed = int(s)
        # move.do_arch = 
        # move.arch = 
        # move.cont = 
        
        request.move_cmds = [move]
        self.future = self.client.call_async(request)

class RobotWorkspaceGraph:
    def __init__(
            self,
            res : int = 800,
            pad : float = 0.05
        ) -> None :

        self.fig = Figure(figsize=(2, 1))
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111)       

        self.res = res
        
        self.current = (None, None)
        self.current_artist = None 

        L12pad = (L1 + L2) * (1 + pad)
        self.xmin = X0 - L12pad
        self.xmax = X0 + L12pad
        self.ymin = Y0 - L12pad
        self.ymax = Y0 + L12pad
        
        self.X = np.linspace(self.xmin, self.xmax, self.res)
        self.Y = np.linspace(self.ymin, self.ymax, self.res)

        self.meshgrid = np.meshgrid(self.X, self.Y, indexing="xy")

        rmin = np.sqrt(L1*L1 + L2*L2 + 2*L1*L2*np.cos(TH2MAX))

        th1=np.linspace(TH1MIN,TH1MAX,1000)
        th2=np.linspace(TH2MIN,TH2MAX,1000)

        temp = L1/L2*np.sin(TH1MAX)
        if (-1<=temp and temp<=1):
            th2 = np.pi - TH1MAX + np.arcsin(temp)

            th2_0tomax=np.linspace(0,th2,1000)
            th2_minto0=np.linspace(-th2,0,1000)
        else:
            th2_0tomax=np.linspace(0,TH2MAX,1000)
            th2_minto0=np.linspace(TH2MIN,0,1000)

        # th2 = 0
        x1 = X0 - (L1*np.sin(th1) + L2*np.sin(th1))
        y1 = Y0 + (L1*np.cos(th1) + L2*np.cos(th1))

        # th1 = TH1MAX
        x2 = X0 - (L1*np.sin(TH1MAX) + L2*np.sin(TH1MAX + th2_0tomax))
        y2 = Y0 + (L1*np.cos(TH1MAX) + L2*np.cos(TH1MAX + th2_0tomax))

        # th1 = TH1MIN
        x3 = X0 - (L1*np.sin(TH1MIN) + L2*np.sin(TH1MIN + th2_minto0))
        y3 = Y0 + (L1*np.cos(TH1MIN) + L2*np.cos(TH1MIN + th2_minto0))

        self.ext_x=np.concatenate((x2[::-1],x1[::-1],x3[::-1]))
        self.ext_y=np.concatenate((y2[::-1],y1[::-1],y3[::-1]))

        self.xc = rmin * np.cos(np.linspace(0,2*np.pi,1000)) + X0
        self.yc = rmin * np.sin(np.linspace(0,2*np.pi,1000)) + Y0

        self.ext_path = Path(np.column_stack([self.ext_x, self.ext_y]))
        self.c_path   = Path(np.column_stack([self.xc, self.yc]))

    def draw(self):
        self.ax.clear()

        self.ax.fill(self.ext_x, self.ext_y, alpha=0.5)
        self.ax.fill(self.xc, self.yc, color="white")

        x, y = self.current
        if x is not None and y is not None:
            (self.current_artist,) = self.ax.plot([x], [y], marker="o", linestyle="None", color="red")

        self.ax.set_xlim(self.xmin, self.xmax)
        self.ax.set_ylim(self.ymin, self.ymax)
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True, linewidth=0.3, color='black', alpha=0.5)

        self.canvas.draw()
    
    def on_click(
            self,
            x : float | None,
            y : float | None
        ) -> None :
        
        if (x==None or y==None) : return
        
        p = [[float(x), float(y)]]

        if not (self.ext_path.contains_points(p)[0] 
            and not self.c_path.contains_points(p)[0]
        ): return

        self.current = (x, y)
        if self.current_artist is None:
            (self.current_artist,) = self.ax.plot([x], [y], "ro")
        else:
            self.current_artist.set_data([x], [y])

        self.canvas.draw_idle()

    def clear_current(self):
        if self.current_artist is not None:
            self.current_artist.remove()
            self.current_artist = None
            self.current = (None, None)
            self.canvas.draw_idle()
            

class CommandGUI(QMainWindow):
    def __init__(
            self, 
            node : CommandNode
        ) -> None :

        super().__init__()

        self.node_ = node
        
        self.setWindowTitle("Command XY GUI")
        self.setFixedSize(QSize(600,540))

        mainLayout = QGridLayout()

        # Plot ===
        
        self.workspace = RobotWorkspaceGraph()
        self.workspace.draw()

        self.cid = self.workspace.canvas.mpl_connect("button_press_event", self.on_click_plot)

        # Sliders ===
        zLayout = QVBoxLayout()
        self.zLabel = QLabel("Z = 0 mm")
        self.zLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.zSldr = QSlider(Qt.Orientation.Vertical)
        self.zSldr.setFixedWidth(100)
        self.zSldr.setRange(ZMIN,ZMAX)
        self.zSldr.setValue(0)
        self.zSldr.valueChanged.connect(
            lambda v :
                self.zLabel.setText(f'Z = {v} mm')
        )
        zLayout.addWidget(self.zSldr, alignment=Qt.AlignmentFlag.AlignHCenter)
        zLayout.addWidget(self.zLabel, alignment=Qt.AlignmentFlag.AlignHCenter)


        thetazLayout = QVBoxLayout()
        self.thetazLabel = QLabel("θ = 0°")
        self.thetazLabel.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.thetazSldr = QSlider(Qt.Orientation.Vertical)
        self.thetazSldr.setFixedWidth(100)
        self.thetazSldr.setRange(THZMIN,THZMAX)
        self.thetazSldr.setValue(0)
        self.thetazSldr.valueChanged.connect(
            lambda v :
                self.thetazLabel.setText(f'θ = {v}°')
        )
        thetazLayout.addWidget(self.thetazSldr, alignment=Qt.AlignmentFlag.AlignHCenter)
        thetazLayout.addWidget(self.thetazLabel, alignment=Qt.AlignmentFlag.AlignHCenter)


        speedLayout = QHBoxLayout()
        self.speedLabel = QLabel("Speed = 20%")
        self.speedSldr = QSlider(Qt.Orientation.Horizontal)
        self.speedSldr.setFixedHeight(50)
        self.speedSldr.setRange(0,100)
        self.speedSldr.setValue(20)
        self.speedSldr.valueChanged.connect(
            lambda v :
                self.speedLabel.setText(f'Speed = {v}%')
        )
        speedLayout.addWidget(self.speedLabel)
        speedLayout.addWidget(self.speedSldr)

        # Buttons ===
        self.sendBtn = QPushButton("Send")
        self.sendBtn.clicked.connect(self.send)
        self.resetBtn = QPushButton("Reset")
        self.resetBtn.clicked.connect(self.reset)
        
        btnLayout = QHBoxLayout()
        btnLayout.addWidget(self.sendBtn)
        btnLayout.addWidget(self.resetBtn)

        # Main Layout ===
        mainLayout.addWidget(self.workspace.canvas,0,0)
        mainLayout.addLayout(zLayout,0,1)
        mainLayout.addLayout(thetazLayout,0,2)
        mainLayout.addLayout(speedLayout,1,0)
        mainLayout.addLayout(btnLayout, 2, 0)
        
        centralWidget = QWidget()
        centralWidget.setLayout(mainLayout)
        self.setCentralWidget(centralWidget)
        return

    def reset(self) -> None :
        self.zSldr.setValue(0)
        self.thetazSldr.setValue(0)
        self.speedSldr.setValue(20)
        self.workspace.clear_current()

    def send(self) -> None :
        x,y = self.workspace.current
        if (x==None or y==None): return
        z = self.zSldr.value()/1000
        thz = self.thetazSldr.value()
        s = self.speedSldr.value()

        self.node_.send_command(x,y,z,thz,s)

    def on_click_plot(
            self,
            event : MouseEvent
        ) -> None :

        self.workspace.on_click(event.xdata,event.ydata)

if __name__=="__main__" :
    rclpy.init()
    node = CommandNode()

    app = QApplication(sys.argv)

    W=CommandGUI(node)
    W.show()
    app.exec()
    