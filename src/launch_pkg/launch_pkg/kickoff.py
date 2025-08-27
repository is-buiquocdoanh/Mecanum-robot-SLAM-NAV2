#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, qos_profile_sensor_data

import launch
import launch.actions
import launch.launch_description_sources
import launch_ros

from message_pkg.msg import *
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Int8

import multiprocessing
import asyncio

# class Ros2LaunchParent:
#     def start(self, launch_description: LaunchDescription):
#         self._stop_event = multiprocessing.Event()
#         self._process = multiprocessing.Process(target=self._run_process, args=(self._stop_event, launch_description), daemon=True)
#         self._process.start()

#     def shutdown(self):
#         self._stop_event.set()
#         self._process.join()

#     def _run_process(self, stop_event, launch_description):
#         loop = asyncio.get_event_loop()
#         launch_service = LaunchService()
#         launch_service.include_launch_description(launch_description)
#         launch_task = loop.create_task(launch_service.run_async())
#         loop.run_until_complete(loop.run_in_executor(None, stop_event.wait))
#         if not launch_task.done():
#             asyncio.ensure_future(launch_service.shutdown(), loop=loop)
#             loop.run_until_complete(launch_task)

class Ros2LaunchParent:
    def start(self, launch_description: launch.LaunchDescription):
        self._stop_event = multiprocessing.Event()
        self._process = multiprocessing.Process(
            target=self._run_process, 
            args=(self._stop_event, launch_description), 
            daemon=True
        )
        self._process.start()

    def shutdown(self):
        if self._process.is_alive():
            self._stop_event.set()
            self._process.join() 

    def _run_process(self, stop_event, launch_description):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        launch_service = launch.LaunchService()
        launch_service.include_launch_description(launch_description)

        async def run_launch():
            launch_task = asyncio.create_task(launch_service.run_async())
            await loop.run_in_executor(None, stop_event.wait)
            if not launch_task.done():
                await launch_service.shutdown()
                await launch_task

        loop.run_until_complete(run_launch())

class Launch:
    def __init__(self, file_launch):
        # -- parameter
        self.fileLaunch = file_launch
        
        # -- variable
        self.process = 0
        self.time_pre = time.time()

    def run_launch_file(self):
        # -- launch
        ld = launch.LaunchDescription()
        ld.add_action(launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(self.fileLaunch)
        ))

        launcher = Ros2LaunchParent()
        launcher.start(ld)

    def start(self):
        if (self.process == 0): # - Launch
            self.run_launch_file()
            self.process = 1  

    def start_and_wait(self, timeWait):
        if (self.process == 0): # - Launch
            self.run_launch_file()
            self.process = 1
            self.time_pre = time.time()
            return 0

        elif (self.process == 1): # - Wait
            t = (time.time() - self.time_pre)%60
            if (t > timeWait):
                self.process = 2
            return 0

        elif (self.process == 2): # - Wait
            return 1

class RobotBringUp(LifecycleNode):
    def __init__(self):
        super().__init__('kickoff')
        self.get_logger().warn("ROS 2 Node RobotBringUp Initialized!")
        self.killnode = 0

        self.declare_parameters(
            namespace='',
            parameters=[
                ('path_checkPort',          ""),
                ('path_nuc',                ""),
                ('path_rtc',                ""),
                ('path_convertCAN',         ""),
                ('path_kinematic',          ""),
                ('path_magline',            ""),
                ('path_rfid',               ""),
                ('path_goalControl',        ""),
                ('path_control',            ""),
                ('path_client',             ""),
                ('path_debug',              ""),
                ('path_reconnect',          ""),
            ]
        )
        
        self.count_node = 0
        self.notification = ''
        self.step = 0
        self.timeWait = 0.4 # s

        # -- Publisher 
        self.pub_stausLaunch = self.create_publisher(StatusLaunch, '/status_launch', 10)
        self.stausLaunch = StatusLaunch()

        # -- Subcriber
        # -- check port
        self.path_checkPort = self.get_parameter('path_checkPort').value
        self.launch_checkPort = Launch(self.path_checkPort)
        self.sub_checkPort = self.create_subscription(
            StatusPort,
            'status_port',
            self.callBack_checkPort,
            10)
        self.sub_checkPort

        self.is_checkPort = 0
        self.count_node += 1

        # -- NUC PC
        self.path_nuc = self.get_parameter('path_nuc').value
        self.launch_nuc = Launch(self.path_nuc)
        self.sub_nuc = self.create_subscription(
            NucInfo,
            'nuc_info',
            self.callBack_nuc,
            10)
        self.sub_nuc

        self.is_nuc = 0
        self.count_node += 1

        # -- reconnectBase
        # -- RTC
        qos_profile_sub = QoSProfile(
			reliability=QoSReliabilityPolicy.RELIABLE,   # Đảm bảo tin nhắn được nhận đầy đủ
			durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # Giữ lại tin nhắn cuối cùng như latched topic trong ROS1
			history=QoSHistoryPolicy.KEEP_LAST,
			depth=10
		)

        self.path_rtc = self.get_parameter('path_rtc').value
        self.launch_rtc = Launch(self.path_rtc)
        self.sub_rtc = self.create_subscription(
            Canreceived,
            'can_received',
            self.callBack_rtc,
            qos_profile_sensor_data)
        self.sub_rtc

        self.is_rtc = 0
        self.count_node += 1

        # -- convert CAN ROS
        self.path_convertCAN = self.get_parameter('path_convertCAN').value
        self.launch_convertCAN = Launch(self.path_convertCAN)
        self.is_convertCAN = 0
        self.count_node += 1

        # -- Kinematic
        # -- 
        self.path_kinematic = self.get_parameter('path_kinematic').value
        self.launch_kinematic = Launch(self.path_kinematic)
        self.sub_kinematic = self.create_subscription(
            McRequest,
            'mc_request',
            self.callBack_kinematic,
            10)
        self.sub_kinematic

        self.is_kinematic = 0
        self.count_node += 1

        # -- Magline
        self.path_magline = self.get_parameter('path_magline').value
        self.launch_magline = Launch(self.path_magline)
        self.sub_magline_front = self.create_subscription(
            MagneticLine,
            'magneticLine_front',
            self.callBack_magline_front,
            10)
        self.sub_magline_front
        self.is_magline_front = 0

        self.sub_magline_behind = self.create_subscription(
            MagneticLine,
            'magneticLine_behind',
            self.callBack_magline_behind,
            10)
        self.sub_magline_behind
        self.is_magline_behind = 0

        self.count_node += 1

        # -- RFID
        self.path_rfid = self.get_parameter('path_rfid').value
        self.launch_rfid = Launch(self.path_rfid)
        self.sub_rfid = self.create_subscription(
            RFID,
            'rfid_respond',
            self.callBack_rfid,
            10)
        self.sub_rfid
        self.is_rfid = 0

        # -- Move Control
        self.path_goalControl = self.get_parameter('path_goalControl').value
        self.launch_goalControl = Launch(self.path_goalControl)
        self.sub_goalControl = self.create_subscription(
            MoveRespond,
            'respond_move',
            self.callBack_goalControl,
            10)
        self.sub_goalControl
        self.is_goalControl = 0

        self.count_node += 1

        # -- stiControl
        self.path_stiControl = self.get_parameter('path_control').value
        self.launch_stiControl = Launch(self.path_stiControl)
        self.sub_stiControl = self.create_subscription(
            NNinfoRespond,
            'NN_infoRespond',
            self.callBack_control,
            10)
        self.sub_stiControl
        self.is_stiControl = 0

        self.count_node += 1

        # -- stiClient
        self.path_stiClient = self.get_parameter('path_client').value
        self.launch_stiClient = Launch(self.path_stiClient)
        self.sub_stiClient = self.create_subscription(
            NNcmdRequest,
            'NN_cmdRequest',
            self.callBack_client,
            10)
        self.sub_stiClient
        self.is_stiClient = 0

        self.count_node += 1

        # -- stiDebug
        self.path_debug = self.get_parameter('path_debug').value
        self.launch_debug = Launch(self.path_debug)
        self.is_debug = 0

        self.count_node += 1

        # -- Recconnect
        self.path_reconnect = self.get_parameter('path_reconnect').value
        self.launch_reconnect = Launch(self.path_reconnect)
        self.is_reconnect = 0

        self.count_node += 1

        # -- Loop
        self.rate = 10
        self.timer_period = 1/self.rate
        self.timer = self.create_timer(self.timer_period, self.run)

    def on_shutdown(self, state):
        self.killnode = 1
        self.get_logger().warn("Shutting down! Exiting program...")
        return TransitionCallbackReturn.SUCCESS

    # -- Callback function
    def callBack_checkPort(self, data):
        self.is_checkPort = 1

    def callBack_rtc(self, data):
        self.is_rtc = 1

    def callBack_kinematic(self, data):
        self.is_kinematic = 1

    def callBack_magline_front(self, data):
        self.is_magline_front = 1

    def callBack_magline_behind(self, data):
        self.is_magline_behind = 1

    def callBack_rfid(self, data):
        self.is_rfid = 1

    def callBack_goalControl(self, data):
        self.is_goalControl = 1

    def callBack_control(self, data):
        self.is_stiControl = 1 

    def callBack_client(self, data):
        self.is_stiClient = 1 

    def callBack_nuc(self, data):
        self.is_nuc = 1 

    # -- Loop
    def run(self):
        # -- checkPort
        if (self.step == 0):
            self.notification = 'launch_checkPort'
            self.launch_checkPort.start()
            if self.is_checkPort == 1:
                self.step += 1
                time.sleep(self.timeWait)

        # -- nuc
        elif (self.step == 1):
            self.notification = 'launch_nuc'
            self.launch_nuc.start()
            if self.is_nuc == 1:
                self.step += 1
                time.sleep(self.timeWait)

        elif (self.step == 2):
            self.notification = 'launch_rtc'
            self.launch_rtc.start()
            if self.is_rtc == 1:
                self.step += 1
                time.sleep(self.timeWait)

        elif (self.step == 3):
            self.notification = 'launch_convertCAN'
            sts = self.launch_convertCAN.start_and_wait(1.5)
            if sts:
                self.step += 1
                time.sleep(self.timeWait)

        elif (self.step == 4):
            self.notification = 'launch_kinematic'
            self.launch_kinematic.start()
            if self.is_kinematic:
                self.step += 1
                time.sleep(self.timeWait)

        elif (self.step == 5):
            self.notification = 'launch_magline'
            self.launch_magline.start()
            if self.is_magline_front and self.is_magline_behind:
                self.step += 1
                time.sleep(self.timeWait)

        elif (self.step == 6):
            self.notification = 'launch_rfid'
            self.launch_rfid.start()
            if self.is_rfid:
                self.step += 1
                time.sleep(self.timeWait)

                # self.step = 12

        elif (self.step == 7):
            self.notification = 'launch_movecontrol'
            self.launch_goalControl.start()
            if self.is_goalControl:
                self.step += 1
                time.sleep(self.timeWait)

        elif (self.step == 8):
            self.notification = 'launch_stiControl'
            self.launch_stiControl.start()
            if self.is_stiControl:
                self.step += 1
                time.sleep(self.timeWait)

        elif (self.step == 9):
            self.notification = 'launch_stiClient'
            sts = self.launch_stiClient.start_and_wait(1.5)
            if self.is_stiClient or sts :
                self.step += 1
                # self.step = 11
                time.sleep(self.timeWait)

        elif (self.step == 10):
            self.notification = 'launch_reconnect'
            sts = self.launch_reconnect.start_and_wait(1.5)
            if self.is_reconnect or sts :
                self.step += 1
                time.sleep(self.timeWait)

        elif (self.step == 11):
            self.notification = 'launch_debug'
            sts = self.launch_debug.start_and_wait(1.5)
            if self.is_debug or sts :
                self.step += 1
                time.sleep(self.timeWait)

        # -- Completed
        elif (self.step == 12):
            # print("Completed Launch")
            self.notification = 'Completed!'

        # -- -- PUBLISH STATUS
        self.stausLaunch.persent = int((self.step/self.count_node)*100.)
        self.stausLaunch.position = self.step
        self.stausLaunch.notification = self.notification

        self.pub_stausLaunch.publish(self.stausLaunch)


        # -- KILL NODE -- 
        if self.killnode:
            sys.exit(0)

def main():
    # -- Khoi tao ROS
    rclpy.init()
    node = RobotBringUp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('Program stopped')

if __name__ == '__main__':
    main()
