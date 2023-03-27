#!/usr/bin/env python
# -*- coding: utf-8 -*-

from ctypes import sizeof
from email import message
from time import sleep
from rcomponent.rcomponent import *


# Insert here general imports:
import math
import threading 

# Insert here msg and srv imports:
from std_msgs.msg import String
from std_msgs.msg import Int16

from robotnik_msgs.msg import StringStamped

from std_srvs.srv import Trigger, TriggerResponse
from robotnik_msgs.srv import SetString, SetStringResponse
from scale_driver.srv import getWeight, getWeightResponse


#TCP/IP
import socket
import sys

class scaleDriver(RComponent):
    """
    driver for TCP/IP
    """


    def __init__(self):

        self.lock = threading.Lock()

        #BIZERBA msgs
        self.readWeight_msg = b"q%\r\n"
        self.tareMachine_msg = b"q\"\r\n"
        #BIZERBA responses
        self.executed_msg = "w0"
        self.no_executed_msg = "w1"
        self.confirmation_msg ="w5"
        RComponent.__init__(self)


    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)
        self.host = rospy.get_param('~server_ip')
        self.port = rospy.get_param('~server_port')



    def ros_setup(self):
        """Creates and inits ROS components"""
        self.timeout = 1.0
        self.timeoutRead = 15.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = (self.host, self.port)
        self.sock.settimeout(self.timeout)
        try:
            self.sock.connect(server_address)
        except:
            rospy.logwarn("Connection refused")

        RComponent.ros_setup(self)

        # Publisher
        self.status_pub = rospy.Publisher('~status', String, queue_size=10)
        self.status_stamped_pub = rospy.Publisher('~status_stamped', StringStamped, queue_size=10)
        self.send_string_tcp = rospy.Service('~send_string', SetString, self.send_string_cb)
        self.read_string_tcp = rospy.Service('~read_string', Trigger, self.read_string_cb)
        self.open_socket_tcp = rospy.Service('~open_connection', Trigger, self.open_connection_cb)
        self.read_weight = rospy.Service('~read_weight', getWeight, self.read_weight_cb)


        return 0


    def init_state(self):
        self.status = String()
        return RComponent.init_state(self)


    def ready_state(self):
        """Actions performed in ready state"""

        # Publish topic with status

        status_stamped = StringStamped()
        status_stamped.header.stamp = rospy.Time.now()
        status_stamped.string = self.status.data

        self.status_pub.publish(self.status)
        self.status_stamped_pub.publish(status_stamped)
        
        return RComponent.ready_state(self)


    def emergency_state(self):
        if(self.check_topics_health() == True):
            self.switch_to_state(State.READY_STATE)


    def shutdown(self):
        self.close_modbus_lock()
        """Shutdowns device
        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """
        self.close_socket_lock()
        return RComponent.shutdown(self)


    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)


    def send_string_cb(self, msg):
        response = SetStringResponse()
        msgData = msg.data
        msgDataDecode =msgData.encode('utf-8')
        msgRN = msgDataDecode+'\r\n'
        self.set_timeout_lock(self.timeoutRead)
        response.ret.success = self.write_socket_lock(msgRN)
        recMsg = "first"
        fullMsg = ""
        while(recMsg != ""):
            recMsg = self.read_socket_lock()
            if(recMsg.find(self.executed_msg) != -1 or recMsg.find(self.executed_msg) != -1):
                self.set_timeout_lock(self.timeout)
            elif(recMsg.find(self.confirmation_msg)!=-1):
                self.set_timeout_lock(self.timeoutRead)
            else:
                self.set_timeout_lock(self.timeout)
            fullMsg = fullMsg + recMsg
        response.ret.message = fullMsg
        return response


    def read_string_cb(self, msg):
        response = TriggerResponse()
        response.message = self.read_socket_lock()
        if response.message == "":
            response.success = False
        else:
            response.success = True
        return response


    def open_connection_cb(self,msg):
        response = TriggerResponse()
        response.success = self.open_socket_lock()
        return response


    def read_weight_cb(self, msg):
        response = getWeightResponse()
        try:
            self.write_socket_lock(self.readWeight_msg)
            data = self.read_socket_lock()
            dataSplit = data.split()
            response.status = dataSplit[0]
            response.weight = float(dataSplit[1].replace("kg", "").replace(",","."))
            response.success = True
        except:
            response.success = False
        return response


    def tare_machine_cb(self, msg):
        response = TriggerResponse()
        try:
            self.write_socket_lock(self.tareMachine_msg)
            data1 = self.read_socket_lock()
            data2 = self.read_socket_lock()
            if data1 == "w0" and data2 == "w5":        
                response.success = True
            else:
                response.success = False
        except:
            response.success = False
        return response


    # Locked TCP/IP functions
    def write_socket_lock(self,msg):
        self.lock.acquire()
        try:
            self.sock.sendall(msg)
            response = True
        except:
            response = False
        self.lock.release()
        return response


    def read_socket_lock(self):
        self.lock.acquire()
        try:
            data = self.sock.recv(1024)
            response = data.decode("utf-8")
            response = response.replace("\n", " ").replace("\r", " ")
        except:
            response = ""
        self.lock.release()
        return response


    def open_socket_lock(self):
        self.lock.acquire()
        self.timeout = 1.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = (self.host, self.port)
        self.sock.settimeout(self.timeout)
        try:
            self.sock.connect(server_address)
            response = True
        except:
            rospy.logwarn("Connection refused")
            response = False
        self.lock.release()
        return response


    def close_socket_lock(self):
        self.lock.acquire()
        try:
            self.sock.close()
            response = True
        except:
            rospy.logwarn("Connection refused")
            response = False
        self.lock.release()
        return response


    def set_timeout_lock(self, value):
        try:
            self.sock.settimeout(value)
            response = True
        except:
            response = False
        return response
