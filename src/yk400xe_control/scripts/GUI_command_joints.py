#!/usr/bin/env python3

import numpy as np
import sys

import rclpy
from rclpy.node import Node

from yk400xe_interfaces.srv import MovepJoints

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

TH1MIN_DEG = -132 # deg
TH1MAX_DEG = 132 # deg

TH2MIN_DEG = -150 # deg
TH2MAX_DEG = 150 # deg

TH1MIN = TH1MIN_DEG * np.pi/180 # rad
TH1MAX = TH1MAX_DEG * np.pi/180 # rad

TH2MIN = TH2MIN_DEG * np.pi/180 # rad
TH2MAX = TH2MAX_DEG * np.pi/180 # rad

ZMIN = 0 # mm
ZMAX = 150 # mm

THZMIN = -360 # deg
THZMAX = 360 # deg

# =============

class CommandNode(Node):
    def __init__(self):
        super().__init__("command_gui_joints_node")

        self.client = self.create_client(MovepJoints,"/command/moveP_joints")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, trying again")
        
    def send_command(
            self,
            j1 : float, 
            j2 : float, 
            j3 : float, 
            j4 : float,
            s : int 
        ) -> None:

        request = MovepJoints.Request()

        request.position = [
            float(j1), 
            float(j2), 
            float(j3), 
            float(j4)
        ]
        request.speed = int(s)
        
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
        
        self.current_links = [(X0,Y0), (X0,Y0+L1), (X0,Y0+L1+L2)]
        self.current_artist_link1 = None 
        self.current_artist_link2 = None 

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

        (self.current_artist_link1,) = self.ax.plot(
            [self.current_links[0][0], self.current_links[1][0]],
            [self.current_links[0][1], self.current_links[1][1]],
            color="red",
            linewidth=2
        )

        (self.current_artist_link2,) = self.ax.plot(
            [self.current_links[1][0], self.current_links[2][0]],
            [self.current_links[1][1], self.current_links[2][1]],
            color="green",
            linewidth=2
        )

        self.ax.set_xlim(self.xmin, self.xmax)
        self.ax.set_ylim(self.ymin, self.ymax)
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True, linewidth=0.3, color='black', alpha=0.5)

        self.canvas.draw()
    
    def update_links(
            self,
            j1 : float,
            j2 : float
        ) -> None :

        self.current_links[1] = (
            X0 - L1*np.sin(j1*np.pi/180),
            Y0 + L1*np.cos(j1*np.pi/180)
        )

        self.current_links[2] = (
            self.current_links[1] [0] - L2*np.sin((j1 + j2)*np.pi/180),
            self.current_links[1] [1] + L2*np.cos((j1 + j2)*np.pi/180)
        )

        if self.current_artist_link1!=None:
            self.current_artist_link1.remove()

        if self.current_artist_link2!=None:
            self.current_artist_link2.remove()

        (self.current_artist_link1,) = self.ax.plot(
            [self.current_links[0][0], self.current_links[1][0]],
            [self.current_links[0][1], self.current_links[1][1]],
            color="red",
            linewidth=2
        )

        (self.current_artist_link2,) = self.ax.plot(
            [self.current_links[1][0], self.current_links[2][0]],
            [self.current_links[1][1], self.current_links[2][1]],
            color="green",
            linewidth=2
        )
        self.canvas.draw_idle()
        

class CommandGUI(QMainWindow):
    def __init__(
            self, 
            node : CommandNode
        ) -> None :

        super().__init__()

        self.node_ = node
        
        self.setWindowTitle("Command Joints GUI")
        self.setFixedSize(QSize(1000,540))

        mainLayout = QGridLayout()

        # Plot ===
        
        self.workspace = RobotWorkspaceGraph()
        self.workspace.draw()

        # Sliders ===
        joint1Layout = QVBoxLayout()
        self.joint1Label = QLabel("θ₁ = 0°")
        self.joint1Label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.joint1Sldr = QSlider(Qt.Orientation.Vertical)
        self.joint1Sldr.setFixedWidth(100)
        self.joint1Sldr.setRange(TH1MIN_DEG,TH1MAX_DEG)
        self.joint1Sldr.setValue(0)
        self.joint1Sldr.valueChanged.connect(self.on_joint1_changed)
        joint1Layout.addWidget(self.joint1Sldr, alignment=Qt.AlignmentFlag.AlignHCenter)
        joint1Layout.addWidget(self.joint1Label, alignment=Qt.AlignmentFlag.AlignHCenter)
        
        joint2Layout = QVBoxLayout()
        self.joint2Label = QLabel("θ₂ = 0°")
        self.joint2Label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.joint2Sldr = QSlider(Qt.Orientation.Vertical)
        self.joint2Sldr.setFixedWidth(100)
        self.joint2Sldr.setRange(TH2MIN_DEG,TH2MAX_DEG)
        self.joint2Sldr.setValue(0)
        self.joint2Sldr.valueChanged.connect(self.on_joint2_changed)
        joint2Layout.addWidget(self.joint2Sldr, alignment=Qt.AlignmentFlag.AlignHCenter)
        joint2Layout.addWidget(self.joint2Label, alignment=Qt.AlignmentFlag.AlignHCenter)
        
        joint3Layout = QVBoxLayout()
        self.joint3Label = QLabel("θ₃ = 0 mm")
        self.joint3Label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.joint3Sldr = QSlider(Qt.Orientation.Vertical)
        self.joint3Sldr.setFixedWidth(100)
        self.joint3Sldr.setRange(ZMIN,ZMAX)
        self.joint3Sldr.setValue(0)
        self.joint3Sldr.valueChanged.connect(
            lambda v :
                self.joint3Label.setText(f'θ₃ = {v} mm')
        )
        joint3Layout.addWidget(self.joint3Sldr, alignment=Qt.AlignmentFlag.AlignHCenter)
        joint3Layout.addWidget(self.joint3Label, alignment=Qt.AlignmentFlag.AlignHCenter)


        joint4Layout = QVBoxLayout()
        self.joint4Label = QLabel("θ₄ = 0°")
        self.joint4Label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.joint4Sldr = QSlider(Qt.Orientation.Vertical)
        self.joint4Sldr.setFixedWidth(100)
        self.joint4Sldr.setRange(THZMIN,THZMAX)
        self.joint4Sldr.setValue(0)
        self.joint4Sldr.valueChanged.connect(
            lambda v :
                self.joint4Label.setText(f'θ₄ = {v}°')
        )
        joint4Layout.addWidget(self.joint4Sldr, alignment=Qt.AlignmentFlag.AlignHCenter)
        joint4Layout.addWidget(self.joint4Label, alignment=Qt.AlignmentFlag.AlignHCenter)


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
        mainLayout.addLayout(joint1Layout,0,1)
        mainLayout.addLayout(joint2Layout,0,2)
        mainLayout.addLayout(joint3Layout,0,3)
        mainLayout.addLayout(joint4Layout,0,4)
        mainLayout.addLayout(speedLayout,1,0)
        mainLayout.addLayout(btnLayout, 2, 0)
        
        centralWidget = QWidget()
        centralWidget.setLayout(mainLayout)
        self.setCentralWidget(centralWidget)
        return

    def reset(self) -> None :
        self.joint1Sldr.setValue(0)
        self.joint2Sldr.setValue(0)
        self.joint3Sldr.setValue(0)
        self.joint4Sldr.setValue(0)
        self.speedSldr.setValue(20)

    def send(self) -> None :
        j1 = self.joint1Sldr.value()*np.pi/180
        j2 = self.joint2Sldr.value()*np.pi/180
        j3 = self.joint3Sldr.value()/1000
        j4 = self.joint4Sldr.value()*np.pi/180
        s = self.speedSldr.value()

        self.node_.send_command(j1,j2,j3,j4,s)

    def on_joint1_changed(
            self,
            v: int
        ) -> None:
        self.joint1Label.setText(f'θ₁ = {v}°')
        self.update_graph()

    def on_joint2_changed(
            self,
            v: int
        ) -> None:
        self.joint2Label.setText(f'θ₂ = {v}°')
        self.update_graph()

    def update_graph(self) -> None:
        j1 = self.joint1Sldr.value()
        j2 = self.joint2Sldr.value()
        self.workspace.update_links(j1,j2)

if __name__=="__main__" :
    rclpy.init()
    node = CommandNode()

    app = QApplication(sys.argv)

    W=CommandGUI(node)
    W.show()
    app.exec()
    