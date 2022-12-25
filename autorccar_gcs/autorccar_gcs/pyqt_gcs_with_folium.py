import sys, io, folium, json
from tracemalloc import start
import rclpy
from rclpy.node import Node
from PyQt5 import QtCore, QtWidgets, QtWebEngineWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from threading import Thread
import folium
from folium.plugins import Draw, MousePosition, marker_cluster
from folium import plugins
from std_msgs.msg import Int8, Int32, Float32, Float32MultiArray, Float64MultiArray
from navigation_interfaces.msg import NavState
import array
import numpy as np
import math
import copy
from user_geometry import *


forceQuit = False
start_coord = [0, 0]
goal_coord = [0, 0]
flagSetGoal = False
flagNavUpdate = False
flagNavUpdateInit = False
posN, posE, posD, velN, velE, velD = 0, 0, 0, 0, 0, 0
Roll, Pitch, Yaw = 0, 0, 0
oriLat, oriLon, oriHei = 37.540022, 127.076111, 0
i = 0
path_llh = []
flagPathUpdate = False

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
        print(msg.data)
        self.publisher_start_goal.publish(msg)

    def path_sub_callback(self, msg):
        global oriLat, oriLon, oriHei
        global path_llh, flagPathUpdate
        print(msg.data)
        
        t = []

        for i in range(11):
            t.append(i*0.1)

        # XX = []
        # YY = []

        path_llh = []

        X = [msg.data[0], msg.data[2], msg.data[4] ,msg.data[6], msg.data[8]]
        Y = [msg.data[1], msg.data[3], msg.data[5] ,msg.data[7], msg.data[9]]

        for i in t:
            tmpX = (1-i)**4*X[0] + 4*(1-i)**3*i*X[1] + 6*(1-i)**2*i**2*X[2] + 4*(1-i)*i**3*X[3] + i**4*X[4]
            tmpY = (1-i)**4*Y[0] + 4*(1-i)**3*i*Y[1] + 6*(1-i)**2*i**2*Y[2] + 4*(1-i)*i**3*Y[3] + i**4*Y[4]
            print(tmpX, tmpY)
            tmpllh = ned2llh([tmpX, tmpY, oriHei], [oriLat*math.pi/180, oriLon*math.pi/180, oriHei])
            path_llh.append([tmpllh[0]* 180/math.pi, tmpllh[1]* 180/math.pi])

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

        Roll = eulr[0]
        Pitch = eulr[1]
        Yaw = eulr[2]

        llh_tmp = xyz2llh([msg.origin.x, msg.origin.y, msg.origin.z])

        # oriLat = msg.origin.x
        # oriLon = msg.origin.y
        # oriHei = msg.origin.z

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
    

class WebEnginePage(QtWebEngineWidgets.QWebEnginePage):

    def javaScriptConsoleMessage(self, level, msg, line, sourceID):
        coords_dict = json.loads(msg)
        coords = coords_dict["geometry"]["coordinates"]
        global goal_coord, flagSetGoal
        goal_coord = [coords[1], coords[0]]
        flagSetGoal = True


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

        self.timer_1 = QTimer(self)
        self.timer_1.start(1000)     # [ms]
        self.timer_1.timeout.connect(self.update_map_with_path)

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

        self.widget_map = self.set_widget_folium()

        labelfont_1 = QLabel().font()
        labelfont_1.setPointSize(12)
        labelfont_1.setBold(True)

        grid_main_1 = QGridLayout()

        grid1_lab = QLabel('Map')
        grid1_lab.setFont(labelfont_1)
        grid1_lab.setAlignment(Qt.AlignCenter)
        grid_main_1.addWidget(self.widget_map, 0, 0)
        grid_main_1.addWidget(QProgressBar(self), 1, 0)

        self.layout_pos = self.set_nav_vlayout('Pos', ['N', 'E', 'D'])
        self.layout_vel = self.set_nav_vlayout('Vel', ['N', 'E', 'D'])
        self.layout_att = self.set_nav_vlayout('Att', ['Roll', 'Pitch', 'Yaw'])

        grid_main_2 = QGridLayout()

        grid2_lab = QLabel('Nav')
        grid2_lab.setFont(labelfont_1)
        grid2_lab.setAlignment(Qt.AlignCenter)

        grid_main_2_1 = QGridLayout()
        grid_main_2_2 = QGridLayout()

        grid_main_2_1.addLayout(self.layout_pos, 0, 0, alignment=Qt.AlignTop)
        grid_main_2_1.addLayout(self.layout_vel, 1, 0, alignment=Qt.AlignTop)
        grid_main_2_1.addLayout(self.layout_att, 2, 0, alignment=Qt.AlignTop)

        btn_setyaw = QPushButton("Set Yaw", self)
        btn_manual = QPushButton("Manual", self)
        btn_start = QPushButton("Start", self)
        btn_stop = QPushButton("Stop", self)
        btn_setyaw.clicked.connect(self.btn_clicked_setyaw)
        btn_manual.clicked.connect(self.btn_clicked_manual)
        btn_start.clicked.connect(self.btn_clicked_start)
        btn_stop.clicked.connect(self.btn_clicked_stop)

        grid_main_2_2.addWidget(btn_setyaw, 0, 0)
        grid_main_2_2.addWidget(btn_manual, 1, 0)
        grid_main_2_2.addWidget(btn_start, 2, 0)
        grid_main_2_2.addWidget(btn_stop, 3, 0)

        grid_main_2.addLayout(grid_main_2_1, 0, 0)
        grid_main_2.addLayout(grid_main_2_2, 1, 0)
        grid_main_2.setRowStretch(0, 1)
        grid_main_2.setRowStretch(1, 1)

        grid_main.addWidget(grid1_lab, 0, 0)
        grid_main.addWidget(grid2_lab, 0, 1)
        grid_main.addLayout(grid_main_1, 1, 0)
        grid_main.addLayout(grid_main_2, 1, 1)
        grid_main.setColumnStretch(0, 8)
        grid_main.setColumnStretch(1, 2)
        
        
    def set_widget_folium(self):
        self.map = folium.Map(
            location=[37.54, 127.076111], tiles="OpenStreetMap", zoom_start=20
        )
        folium.LatLngPopup().add_to(self.map)
        MousePosition().add_to(self.map)
        folium_draw = Draw(
            draw_options={
                "polyline": False,
                "rectangle": False,
                "polygon": False,
                "circle": True,
                "marker": True,
                "circlemarker": True,
            },
            edit_options={"edit": False},
        )
        self.map.add_child(folium_draw)
        data = io.BytesIO()
        self.map.save(data, close_file=False)

        w = QtWebEngineWidgets.QWebEngineView()
        page = WebEnginePage(w)
        w.setPage(page)
        w.showFullScreen()
        w.setHtml(data.getvalue().decode())

        return w

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
    
    def btn_clicked_setyaw(self):
        text, ok = QInputDialog.getText(self, 'Set Yaw', 'Enter the yaw angle relative to true north:')
        if ok:
            yaw = float(text)
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
        global flagSetGoal, start_coord, goal_coord
        global flagNavUpdate, posN, posE, posD, velN, velE, velD, flagNavUpdateInit
        global Roll, Pitch, Yaw
        global oriLat, oriLon, oriHei

        if flagSetGoal:
            print(goal_coord[0], goal_coord[1])
            print(oriLat, oriLon)
            goal_ned = llh2ned([goal_coord[0]*math.pi/180, goal_coord[1]*math.pi/180, oriHei],[oriLat*math.pi/180, oriLon*math.pi/180, oriHei])
            goal_ned = [0, 10, oriHei]
            print(goal_ned[0], goal_ned[1])
            self.ros2_node.publish_start_goal(start_coord, goal_ned)
            flagSetGoal = False

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
                self.map = folium.Map(
                    location=[oriLat, oriLon], tiles="OpenStreetMap", zoom_start=20
                )
                folium.LatLngPopup().add_to(self.map)
                MousePosition().add_to(self.map)
                folium_draw = Draw(
                    draw_options={
                    "polyline": False,
                    "rectangle": False,
                    "polygon": False,
                    "circle": True,
                    "marker": True,
                    "circlemarker": True,
                    },
                    edit_options={"edit": False},
                )
                self.map.add_child(folium_draw)
                data = io.BytesIO()
                self.map.save(data, close_file=False)

                self.widget_map = QtWebEngineWidgets.QWebEngineView()
                page = WebEnginePage(self.widget_map)
                self.widget_map.setPage(page)
                self.widget_map.showFullScreen()
                self.widget_map.setHtml(data.getvalue().decode())

                flagNavUpdateInit = True

            flagNavUpdate = False

    def update_map_with_path(self):
        global path_llh, flagPathUpdate
        
        if(flagPathUpdate):
            for i in range(11):
                folium.CircleMarker(location=[path_llh[i][0], path_llh[i][1]], radius=10, color='red').add_to(self.map)

            self.widget_map.update()

            data = io.BytesIO()
            self.map.save(data, close_file=False)

            w = QtWebEngineWidgets.QWebEngineView()
            page = WebEnginePage(w)
            w.setPage(page)
            self.widget_map.showFullScreen()
            self.widget_map.setHtml(data.getvalue().decode())

            flagPathUpdate = False

    def update_map_with_current_pos(self):
        pass
        # global PosN, PosE
        # folium.CircleMarker(location=[37.540022, 127.076111], radius=10, color='red').add_to(self.map)
        
        # self.widget_map.update()

        # data = io.BytesIO()
        # self.map.save(data, close_file=False)

        # self.widget_map.

        # w = QtWebEngineWidgets.QWebEngineView()
        # page = WebEnginePage(w)
        # w.setPage(page)
        # self.widget_map.showFullScreen()
        # self.widget_map.setHtml(data.getvalue().decode())

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