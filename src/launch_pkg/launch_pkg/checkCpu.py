#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author : Phùng Quý Dương - Archie Phùng
# Date: 10/7/2025

"""
    > Get CPU Info
	> Get Wifi Info
	> Get time run of NUC after boot
	> Auto connect wifi if it get lost
	> Send number of losting wifi to Debug file
"""
import sys
import os
import time
import subprocess
import re
import psutil
import netifaces
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from message_pkg.msg import NucInfo
from std_msgs.msg import Int16

class NUC_Info(LifecycleNode):
    def __init__(self):
        super().__init__('NUC_info')
        self.get_logger().info("ROS 2 Node Initialized!")
        self.killnode = 0
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('name_card', 'wlo1'),
                ('name_eth', 'enp114s0'),
                ('address_traffic', '172.21.15.224'),
            ]
        )
        
        self.name_card = self.get_parameter('name_card').value
        self.name_eth = self.get_parameter('name_eth').value
        self.address_traffic = self.get_parameter('address_traffic').value
        
        self.pub_NUC = self.create_publisher(NucInfo, '/nuc_info', 10)

        self.rate = 10
        self.timer_period = 1/self.rate
        self.timer = self.create_timer(self.timer_period, self.run)

        self.NUC_info = NucInfo()

        self.time_read = 0.5
        self.runOnce = 0
        self.start_time = time.time()
        self.step = 1
        self.time_checkWifi = time.time()
        self.uptime = 0
        
    def get_hostname(self):
        try:
            output = os.popen("hostname").read()
            # print ("output: ", output)
            leng = len(output)
            hostname = str(output)[0:leng-1]
            # print ("hostname: ", hostname)
            return hostname
        except Exception:
            return "-1"

    def get_MAC(self, name_card): # name_card : str()
        try:
            MAC = ''
            output = os.popen("ip addr show {}".format(name_card) ).read()
            pos1 = str(output).find('link/ether ') # tuyet doi ko sua linh tinh.
            pos2 = str(output).find(' brd')   # tuyet doi ko sua linh tinh.

            if (pos1 >= 0 and pos2 > 0):
                MAC = str(output)[pos1+11:pos2]

            # print ("MAC: ", MAC)
            return MAC
        except Exception:
            return "-1"

    def get_ipAuto(self, name_card): # name_card : str()
        try:
            address = re.search(re.compile(r'(?<=inet )(.*)(?=\/)', re.M), os.popen("ip addr show {}".format(name_card) ).read()).groups()[0]
            # print ("address: ", address)
            return address
        except Exception:
            return "-1"

    def get_ethernet_ip(self,interface):
        try:
            addrs = netifaces.ifaddresses(interface)
            ip_info = addrs[netifaces.AF_INET][0]
            ip_address = ip_info['addr']
            return str(ip_address)
        except (KeyError, IndexError):
            return "-1"
                  
    def get_cpu_usage(self, time_read):
        try:
            cpu_usage = psutil.cpu_percent(time_read)
            return cpu_usage
        except Exception:
            return 0
        
    def get_cpu_temp(self):
        try:
            temperature = psutil.sensors_temperatures().get('coretemp')[0].current
            return temperature
        except Exception:
            return 0

    def get_ram_usage(self):
        """
        Obtains the absolute number of RAM bytes currently in use by the system.
        :returns: System RAM usage in bytes.
        :rtype: int
        """
        try:
            ram_usage = int(psutil.virtual_memory().total - psutil.virtual_memory().available)
            return ram_usage
        except Exception:
            return 0
        
    def get_ram_total(self):
        """
        Obtains the total amount of RAM in bytes available to the system.
        :returns: Total system RAM in bytes.
        :rtype: int
        """
        try:
            ram_total = int(psutil.virtual_memory().total)
            return ram_total
        except Exception:
            return 0
        

    def get_ram_usage_pct(self):
        """
        Obtains the system's current RAM usage.
        :returns: System RAM usage as a percentage.
        :rtype: float
        """
        try:
            ram_percent = psutil.virtual_memory().percent
            return ram_percent
        except Exception:
            return 0

    def get_qualityWifi(self, name_card): # int
        try:
            pos_quality = '0'
            pos_signal = '0'
            output = os.popen("iwconfig {}".format(name_card)).read()
            # print(output)
            pos_quality = str(output).find('Link Quality=')
            pos_signal = str(output).find('Signal level=')
            # pos_bitrate = str(output).find('Bit Rate=')
            # pos_txpower = str(output).find('Tx-Power=')
            # -
            if pos_quality >= 0:
                quality_data = str(output)[pos_quality+13:pos_quality+15]
            # print ("quality_data: ", int(quality_data))
            
            # -
            if pos_signal >= 0:
                signal_data = str(output)[pos_signal+13:pos_signal+16]
            # print ("signal_data: ", int(signal_out)) 

            # -
            # if pos_bitrate >= 0:
            #     bitrate_data = str(output)[pos_bitrate+13:pos_bitrate+16]
            # # print ("bitrate_data: ", int(bitrate_data)) 

            # # -
            # if pos_txpower >= 0:
            #     txpwower_data = str(output)[pos_txpower+13:pos_txpower+16]
            # print ("txpwower_data: ", int(txpwower_data)) 

            return int(quality_data), int(signal_data)
        except Exception:
            return 0, 0

    def get_AccessPointMAC(self, name_card): # int
        try:
            ap_mac = ''
            ap_mac_info = ''
            output = os.popen("iwconfig {}".format(name_card)).read()
            # print(output)
            ap_mac = str(output).find('Access Point:')
            # -
            ap_mac_info = str(output)[ap_mac+14:ap_mac+31]

            # print ("quality_data: ", int(quality_data))

            return ap_mac_info
        
        except Exception:
            return ''

    def ping_traffic(self, address):
        try:
            ping = subprocess.check_output("ping -c 1 -w 1 {}".format(address), shell=True)
            # print(ping)
            vitri = str(ping).find("time")
            time_ping = str(ping)[(vitri+5):(vitri+9)]
            # print (time_ping)
            return str(float(time_ping))
        except Exception:
            return '-1'
        
    def convert_intTotime(self, time):
        str_time = ""
        time_hour = int(time/3600)
        time = time - time_hour*3600
        time_minute = int(time/60)
        time_second = time - time_minute*60

        if time_hour == 0:
            if time_minute == 0:
                str_time = str(time_second) + "s"
            else:
                str_time = str(time_minute) + "m" + str(time_second) + "s"
        else:
            str_time = str(time_hour) + "h" + str(time_minute) + "m" + str(time_second) + "s"
            
        return str_time

    def run(self):
        # -- NUC mac
        self.NUC_info.nuc_mac = self.get_MAC(self.name_card)

        # -- NUC name
        self.NUC_info.nuc_name = self.get_hostname()

        # -- NUC IP Wifi
        self.NUC_info.nuc_ipwifi = self.get_ipAuto(self.name_card)

        # -- NUC IP Ethernet
        self.NUC_info.nuc_ipethernet = self.get_ethernet_ip(self.name_eth)

        # -- CPU usage
        self.NUC_info.cpu_usage = self.get_cpu_usage(self.time_read)

        # -- CPU temp
        self.NUC_info.cpu_temp = self.get_cpu_temp()

        # -- RAM usage
        self.NUC_info.ram_usage = int(self.get_ram_usage() / 1024 / 1024)
        self.NUC_info.ram_usage = round(self.NUC_info.ram_usage/1000, 1)

        # -- RAM total
        self.NUC_info.ram_total = int(self.get_ram_total() / 1024 / 1024)
        self.NUC_info.ram_total = round(self.NUC_info.ram_total/1000, 1)
        
        # -- RAM percent
        self.NUC_info.ram_percent = self.get_ram_usage_pct()

        # -- Wifi
        self.NUC_info.wifi_quality, self.NUC_info.wifi_signal = self.get_qualityWifi(self.name_card)

        # -- Access point 
        self.NUC_info.ap_mac = self.get_AccessPointMAC(self.name_card)
        # if self.step == 1:		
        #     if self.NUC_info.wifi_quality == 0 or self.NUC_info.wifi_signal == 0:
        #         if self.runOnce == 1:
        #             self.runOnce = 2
        #             self.start_time = time.time()
        #             os.system("nmcli radio wifi off")
        #             self.time_checkWifi = time.time()
                    
        #         elif self.runOnce == 2:
        #             if time.time() - self.start_time > 1:
        #                 os.system("nmcli radio wifi on")
        #                 self.step = 2
        #                 self.runOnce = 0
        #     else:
        #         self.runOnce = 1

        # elif self.step == 2:              # check trạng thái đá kết nối được wifi chưa sau 5s
        #     if time.time() - self.time_checkWifi > 5:
        #         if self.NUC_info.wifi_quality == 0 or self.NUC_info.wifi_signal == 0:
        #             self.runOnce = 1
        #             self.step = 1
                    
        #     if self.NUC_info.wifi_quality != 0 and self.NUC_info.wifi_signal != 0:
        #         self.step = 1
        #         self.runOnce = 1

        # -- Ping server
        self.NUC_info.ping_server = self.ping_traffic(self.address_traffic)

        # -- Time run
        if time.time() - self.start_time >= 1:
            self.start_time = time.time()
            self.uptime = self.uptime + 1
        
        self.NUC_info.uptime = self.convert_intTotime(self.uptime)

        self.pub_NUC.publish(self.NUC_info)

        # -- KILL NODE -- 
        if self.killnode:
            sys.exit(0)

def main():
    rclpy.init()
    node = NUC_Info()
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