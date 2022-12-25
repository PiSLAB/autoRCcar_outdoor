import sys, io, json
from tracemalloc import start
import rclpy
from rclpy.node import Node
#from PyQt5 import QtCore, QtWidgets, QtWebEngineWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from threading import Thread
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
from std_msgs.msg import Int8, Int32, Float32, Float32MultiArray, Float64MultiArray
from autorccar_interfaces.msg import NavState
import array
import numpy as np
import math
import copy
from .submodules.user_geometry import *


forceQuit = False
start_coord = [0, 0]
goal_coord = [0, 0]
flagNavUpdate = False
flagNavUpdateInit = False
posN, posE, posD, velN, velE, velD = 0, 0, 0, 0, 0, 0
Roll, Pitch, Yaw = 0, 0, 0
oriLat, oriLon, oriHei = 37.540022, 127.076111, 0
i = 0
path_ned = []

flagPathUpdate = False
flagPathPosUpdate = False

class Ros2Node(Node):
    def __init__(self, node_name):

        super().__init__(node_name)

        self.publisher_command = self.create_publisher(Int8, 'command_topic', 10)
        self.publisher_setyaw = self.create_publisher(Float32, 'setyaw_topic', 10)
        self.publisher_start_goal = self.create_publisher(Float64MultiArray, 'start_goal_topic', 10)
        self.subscription_path = self.create_subscription(Float64MultiArray, 'path_topic', self.path_sub_callback, 10)
        self.subscription_nav = self.create_subscription(NavState, 'nav_topic', self.nav_sub_callback, 10)

    def publish_command(self, command):
        msg = Int8()
        msg.data = command
        self.get_logger().info('pub-command: "%d"' % msg.data)
        self.publisher_command.publish(msg)

    def publish_setyaw(self, yaw):
        msg = Float32()
        msg.data = yaw
        self.get_logger().info('pub-setYaw: "%lf"' % msg.data)
        self.publisher_setyaw.publish(msg)

    def publish_start_goal(self, start, goal):
        msg = Float64MultiArray()
        msg.data.append(start[0])
        msg.data.append(start[1])
        msg.data.append(goal[0])
        msg.data.append(goal[1])
        self.publisher_start_goal.publish(msg)

    def path_sub_callback(self, msg):
        global oriLat, oriLon, oriHei
        global path_ned, flagPathUpdate
        
        t = []

        for i in range(101):
            t.append(i*0.01)

        path_ned = []

        X = [msg.data[0], msg.data[2], msg.data[4] ,msg.data[6], msg.data[8]]
        Y = [msg.data[1], msg.data[3], msg.data[5] ,msg.data[7], msg.data[9]]

        for i in t:
            tmpX = (1-i)**4*X[0] + 4*(1-i)**3*i*X[1] + 6*(1-i)**2*i**2*X[2] + 4*(1-i)*i**3*X[3] + i**4*X[4]
            tmpY = (1-i)**4*Y[0] + 4*(1-i)**3*i*Y[1] + 6*(1-i)**2*i**2*Y[2] + 4*(1-i)*i**3*Y[3] + i**4*Y[4]
            path_ned.append([tmpX, tmpY])

        flagPathUpdate = True
    
    def nav_sub_callback(self, msg):
        # self.get_logger().info('sub-nav: "%d"' % msg.data)
        global posN, posE, posD, velN, velE, velD
        global Roll, Pitch, Yaw
        global oriLat, oriLon, oriHei
        global flagNavUpdate

        posN = msg.position.x
        posE = msg.position.y
        posD = msg.position.z

        velN = msg.velocity.x
        velE = msg.velocity.y
        velD = msg.velocity.z

        qx = msg.quaternion.x
        qy = msg.quaternion.y
        qz = msg.quaternion.z
        qw = msg.quaternion.w

        eulr = quat2eulr([qw, qx, qy, qz])

        Roll = eulr[0] * 180/math.pi
        Pitch = eulr[1] * 180/math.pi
        Yaw = eulr[2] * 180/math.pi

        llh_tmp = xyz2llh([msg.origin.x, msg.origin.y, msg.origin.z])

        oriLat = llh_tmp[0]*180/math.pi
        oriLon = llh_tmp[1]*180/math.pi
        oriHei = llh_tmp[2]
        
        flagNavUpdate = True



def WindowStyle(app):
    app.setStyle("Fusion")
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.white)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)
    

class GCSWindow(QMainWindow):

    def __init__(self, ros2_node):
        super().__init__()
        self.setWindowTitle("GCS")
        self.window_width, self.window_height = 1028, 720
        self.setMinimumSize(self.window_width, self.window_height)

        self.initUI()

        self.ros2_node = ros2_node
        self.ros2_thread = Thread(target=rclpy.spin, args=(self.ros2_node, ))
        self.ros2_thread.start()

        self.timer_0 = QTimer(self)
        self.timer_0.start(100)     # [ms]
        self.timer_0.timeout.connect(self.data_update_check)      
        self.timer_0.timeout.connect(self.update_graph_with_current_pos) 

        self.timer_1 = QTimer(self)
        self.timer_1.start(1000)     # [ms]
        self.timer_1.timeout.connect(self.update_graph_with_path)

    def initUI(self):

        tabs = QTabWidget()
        tab1 = QWidget()
        tab2 = QWidget()
        tab3 = QWidget()

        tabs.addTab(tab1, "main")
        tabs.addTab(tab2, "details")
        tabs.addTab(tab3, "param")

        self.setCentralWidget(tabs)

        grid_main = QGridLayout()

        tab1.setLayout(grid_main)

        self.widget_qtgraph = self.set_widget_qtgraph()
        self.widget_qtgraph.scene().sigMouseClicked.connect(self.qtgraph_mouse_clicked)

        labelfont_1 = QLabel().font()
        labelfont_1.setPointSize(12)
        labelfont_1.setBold(True)

        grid_main_1 = QGridLayout()

        grid1_lab = QLabel('Graph')
        grid1_lab.setFont(labelfont_1)
        grid1_lab.setAlignment(Qt.AlignCenter)
        grid_main_1.addWidget(self.widget_qtgraph, 0, 0)
        grid_main_1.addWidget(QProgressBar(self), 1, 0)

        self.layout_pos = self.set_nav_vlayout('Pos', ['N', 'E', 'D'])
        self.layout_vel = self.set_nav_vlayout('Vel', ['N', 'E', 'D'])
        self.layout_att = self.set_nav_vlayout('Att', ['Roll', 'Pitch', 'Yaw'])

        layout_goal_set = self.set_setting_layout()

        grid_main_2 = QGridLayout()

        grid2_lab = QLabel('Nav')
        grid2_lab.setFont(labelfont_1)
        grid2_lab.setAlignment(Qt.AlignCenter)

        grid_main_2_1 = QGridLayout()
        grid_main_2_2 = QGridLayout()
        grid_main_2_3 = QGridLayout()

        grid_main_2_1.addLayout(self.layout_pos, 0, 0, alignment=Qt.AlignTop)
        grid_main_2_1.addLayout(self.layout_vel, 1, 0, alignment=Qt.AlignTop)
        grid_main_2_1.addLayout(self.layout_att, 2, 0, alignment=Qt.AlignTop)


        btn_setgoal = QPushButton("Set Goal", self)
        btn_setyaw = QPushButton("Set Yaw", self)
        btn_setgoal.clicked.connect(self.btn_clicked_setgoal)
        btn_setyaw.clicked.connect(self.btn_clicked_setyaw)

        grid_main_2_2.addLayout(layout_goal_set, 0, 0)
        grid_main_2_2.addWidget(btn_setgoal, 1, 0)
        grid_main_2_2.addWidget(btn_setyaw, 2, 0)

        btn_manual = QPushButton("Manual", self)
        btn_start = QPushButton("Start", self)
        btn_stop = QPushButton("Stop", self)
        btn_manual.clicked.connect(self.btn_clicked_manual)
        btn_start.clicked.connect(self.btn_clicked_start)
        btn_stop.clicked.connect(self.btn_clicked_stop)

        grid_main_2_3.addWidget(btn_manual, 2, 0)
        grid_main_2_3.addWidget(btn_start, 3, 0)
        grid_main_2_3.addWidget(btn_stop, 4, 0)

        grid_main_2.addLayout(grid_main_2_1, 0, 0)
        grid_main_2.addLayout(grid_main_2_2, 1, 0)
        grid_main_2.addLayout(grid_main_2_3, 2, 0)
        grid_main_2.setRowStretch(0, 1)
        grid_main_2.setRowStretch(1, 1)
        grid_main_2.setRowStretch(2, 1)

        grid_main.addWidget(grid1_lab, 0, 0)
        grid_main.addWidget(grid2_lab, 0, 1)
        grid_main.addLayout(grid_main_1, 1, 0)
        grid_main.addLayout(grid_main_2, 1, 1)
        grid_main.setColumnStretch(0, 7)
        grid_main.setColumnStretch(1, 3)
        

    def set_widget_qtgraph(self):
        graph = pg.PlotWidget()

        graph.setTitle("Position")
        graph.setLabel("left","North[m]")
        graph.setLabel("bottom","East[m]")
        graph.showGrid(x=True, y=True)
        graph.addLegend()

        graph.setRange(rect=None, xRange=(-10, 10), yRange=(-10, 10))

        return graph

    def qtgraph_mouse_clicked(self, evt):
        global start_coord, goal_coord

        vb = self.widget_qtgraph.plotItem.vb
        scene_coords = evt.scenePos()
        if self.widget_qtgraph.sceneBoundingRect().contains(scene_coords):
            mouse_point = vb.mapSceneToView(scene_coords)
            if(evt.button() == 1):
                print('set goal')
                goal_coord = [mouse_point.y(), mouse_point.x()]
                self.goal_setting_n.setText(str(round(goal_coord[0],3)))
                self.goal_setting_e.setText(str(round(goal_coord[1],3)))
                self.ros2_node.publish_start_goal(start_coord, goal_coord)

    def set_nav_vlayout(self, title, str):

        titlefont = QLabel().font()
        titlefont.setBold(True)

        title_lab = QLabel(title)
        title_lab.setFont(titlefont)

        layout = QGridLayout()
        layout.addWidget(title_lab, 0, 0)
        layout.addWidget(QLabel(str[0]), 1, 0)
        layout.addWidget(QLineEdit('0.0000'), 1, 1)
        layout.addWidget(QLabel(str[1]), 2, 0)
        layout.addWidget(QLineEdit('0.0000'), 2, 1)
        layout.addWidget(QLabel(str[2]), 3, 0)
        layout.addWidget(QLineEdit('0.0000'), 3, 1)

        return layout

    def set_setting_layout(self):

        titlefont = QLabel().font()
        titlefont.setPointSize(12)
        titlefont.setBold(True)

        title_lab = QLabel("Setting")
        title_lab.setFont(titlefont)
        title_lab.setAlignment(Qt.AlignCenter)

        self.goal_setting_n = QLineEdit('0.0')
        self.goal_setting_e = QLineEdit('0.0')

        layout = QGridLayout()
        layout.addWidget(title_lab, 0, 2)
        layout.addWidget(QLabel('N:'), 1, 0)
        layout.addWidget(self.goal_setting_n, 1, 1)
        layout.addWidget(QLabel('E:'), 1, 2)
        layout.addWidget(self.goal_setting_e, 1, 3)

        layout.setRowStretch(0,1)
        layout.setRowStretch(1,3)

        layout.setColumnStretch(0,1)
        layout.setColumnStretch(1,4)
        layout.setColumnStretch(2,1)
        layout.setColumnStretch(3,4)

        return layout
    
    def btn_clicked_setgoal(self):
        global goal_coord

        goal_coord[0] = float(self.goal_setting_n.text())
        goal_coord[1] = float(self.goal_setting_e.text())

        print('set goal')
        self.ros2_node.publish_start_goal(start_coord, goal_coord)


    def btn_clicked_setyaw(self):
        val, ok = QInputDialog.getDouble(self, 'Set Yaw', 'Enter the yaw angle relative to true north:')
        if ok:
            yaw = val
            while(yaw > 180):
                yaw = yaw - 360
            while(yaw <= -180):
                yaw = yaw + 360
            print('set yaw')
            self.ros2_node.publish_setyaw(yaw)

    def btn_clicked_manual(self):
        print('manual')
        self.ros2_node.publish_command(2)

    def btn_clicked_start(self):
        print('start')
        self.ros2_node.publish_command(1)

    def btn_clicked_stop(self):
        print('stop')
        self.ros2_node.publish_command(0)

    def data_update_check(self):
        global flagNavUpdate, posN, posE, posD, velN, velE, velD, flagNavUpdateInit
        global Roll, Pitch, Yaw
        global oriLat, oriLon, oriHei

        if flagNavUpdate:
            self.layout_pos.itemAt(2).widget().setText(str(posN))
            self.layout_pos.itemAt(4).widget().setText(str(posE))
            self.layout_pos.itemAt(6).widget().setText(str(posD))
            
            self.layout_vel.itemAt(2).widget().setText(str(velN))
            self.layout_vel.itemAt(4).widget().setText(str(velE))
            self.layout_vel.itemAt(6).widget().setText(str(velD))

            self.layout_att.itemAt(2).widget().setText(str(Roll))
            self.layout_att.itemAt(4).widget().setText(str(Pitch))
            self.layout_att.itemAt(6).widget().setText(str(Yaw))

            if flagNavUpdateInit == False:
                
                
                flagNavUpdateInit = True

            flagNavUpdate = False

    def update_graph_with_path(self):
        global path_ned, flagPathUpdate, flagPathPosUpdate

        if(flagPathUpdate):

            flagPathPosUpdate = False
            self.widget_qtgraph.clear()
            
            X = []
            Y = []
            for ii in range(101):
                X.append(path_ned[ii][1])
                Y.append(path_ned[ii][0])
            
            self.widget_qtgraph.plot(x=X, y=Y, pen=pg.mkPen(width=2, color='y'), name="path")
            self.widget_qtgraph.plot(x=[X[0]], y=[Y[0]], symbol='o', symbolSize=20, symbolBrush='g', name="start")
            self.widget_qtgraph.plot(x=[X[100]], y=[Y[100]], symbol='o', symbolSize=20, symbolBrush='r', name="goal")
            self.pospl = self.widget_qtgraph.plot(pen=pg.mkPen(width=2, color='w'), name="pos")
            self.curpos = self.widget_qtgraph.plot(symbol='o', symbolSize=20, symbolBrush='w', name="cur_pos")
            self.pospl_pos_n = []
            self.pospl_pos_e = []
            self.cnt = 0
            flagPathPosUpdate = True

            flagPathUpdate = False

    def update_graph_with_current_pos(self):
        global posN, posE, path_ned, flagPathPosUpdate

        if(flagPathPosUpdate == True):

            self.pospl_pos_n.append(posN)
            self.pospl_pos_e.append(posE)

            self.curpos.setData(x=[posE], y=[posN])
            self.pospl.setData(x=self.pospl_pos_e, y=self.pospl_pos_n)
        

    def closeEvent(self, event):
        print('Close window')
        rclpy.shutdown()
        self.ros2_thread.join()
        global forceQuit
        forceQuit = True


def main(args=None):

    app = QtWidgets.QApplication(sys.argv)
    WindowStyle(app)

    rclpy.init(args=args)

    node = Ros2Node('pyqt_gcs')

    gcsapp = GCSWindow(node)
    gcsapp.show()

    try:
        app.exec_()    # 이벤트 루프 시작
    except forceQuit:
        app.quit()



if __name__ == '__main__':
    main()
