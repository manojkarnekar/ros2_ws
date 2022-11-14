#!/usr/bin/env python

import time
import serial

class Test():

    def __init__(self):
        try:
            self.device_name = "/dev/electron"
            self.baud_rate = 9600
            self.ser = serial.Serial(self.device_name, self.baud_rate)
            self.send_command = False
        except Exception as e:
            print(e)
        else:
            print("init successful")

    def spin(self):
        while(True):
            if self.ser.in_waiting > 0:
                self.serial_Command = self.ser.read()
                print(self.serial_Command)
                if self.serial_Command == 'T':
                    print("T Received")

if __name__ == '__main__':
    Test_1 = Test()
    Test_1.spin()