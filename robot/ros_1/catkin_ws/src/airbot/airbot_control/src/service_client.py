#!/usr/bin/env python

import rospy
import time
import serial
import signal
from std_msgs.msg import Int8
from std_srvs.srv import Trigger, TriggerResponse

class Test():

    def __init__(self):
        try:
            rospy.init_node('service_client')
            rospy.Subscriber('airbot_motion_state', Int8, self.motion_state_cb, queue_size=1)
            rospy.Service('airbot/complete_stop', Trigger, self.complete_stop_handler)
            rospy.wait_for_service('airbot/start_autonomous_motion')
            rospy.wait_for_service('airbot/stop_autonomous_motion')
            self.call_start = rospy.ServiceProxy('airbot/start_autonomous_motion', Trigger)
            self.call_stop = rospy.ServiceProxy('airbot/stop_autonomous_motion', Trigger)
            self.rate = rospy.get_param('rate', 15.0)
            self.cycle_complete = False
            self.moving_started = False
            self.device_name = "/dev/electron"
            self.baud_rate = 9600
            self.ser = serial.Serial(self.device_name, self.baud_rate)
            signal.signal(signal.SIGINT, self.handler)
        except Exception as e:
            rospy.loginfo(e)
        else:
            rospy.loginfo("{0} node initialized successfully".format(rospy.get_name()))

    def handler(self, signum, frame):
        self.res = input("Ctrl-c was pressed. Do you really want to exit? y(1)/n(0)")
        self.res = int(self.res)
        if self.res == 1:
            self.cycle_complete = True

    def millis(self):
        return round(time.time() * 1000)

    def motion_state_cb(self, state):
        if state.data == 2:
            self.cycle_complete = True

    def complete_stop_handler(self, req):
        res = TriggerResponse()
        self.cycle_complete = True
        res.success = True
        res.message = "Stopping Cycle"
        return res

    def spin(self):
        rate = rospy.Rate(self.rate)
        Temp_time = self.millis()
        self.ser.write("S")
        print("Starting Cycle")
        while not rospy.is_shutdown():
            try:
                if self.ser.in_waiting > 0:
                    self.serial_Command = self.ser.read()
                    if self.serial_Command == 'T':
                        self.cycle_complete = True

                if self.moving_started:
                    if self.millis() - Temp_time > 5000:
                        success = self.call_stop()
                        print(success)
                        self.moving_started = False
                        Temp_time = self.millis()
                else:
                    if self.millis() - Temp_time > 10000:
                        success = self.call_start()
                        print(success)
                        self.moving_started = True
                        Temp_time = self.millis()

                if self.cycle_complete:
                    print(self.cycle_complete)
                    self.ser.write("T")
                    success = self.call_stop()
                    print(success)
                    print("Cycle Complete")
                    exit()

                rate.sleep()

            except rospy.ROSInterruptException:
                self.ser.write("T")
                success = self.call_stop()
                print("Cycle Interrupted")
                return

if __name__ == '__main__':
    Test_1 = Test()
    Test_1.spin()