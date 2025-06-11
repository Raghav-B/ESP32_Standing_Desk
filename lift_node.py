#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import serial
import threading
import time
import os

LIFT_LAST_HEIGHT_SAVE_FILE = os.path.join(os.path.expanduser('~'), "Iron/lift_last_height.save")
LIFT_MAX_HEIGHT_SAVE_FILE = os.path.join(os.path.expanduser('~'), "Iron/lift_max_height.save")

# These are messages receivable by this ROS node from the lift microcontroller
class Arduino2NodeCommand:
    OPTICAL_SENSOR_BLOCKED = 97
    OPTICAL_SENSOR_UNBLOCKED = 98
    CURRENT_LIMIT_TRIGGERED = 99
    CURRENT_LIMIT_UNTRIGGERED = 100
    ROS_CMD_COMPLETE = 101
    CUR_HEIGHT = 102
    MAX_HEIGHT = 103
    GOAL_HEIGHT = 104
    OPTICAL_SENSOR_OFF = 105
    END_CALIBRATION = 106


# These are messages sendable by this ROS node to the lift microcontroller
class Node2ArduinoCommand:
    SAVED_LAST_HEIGHT = 97 # a
    SAVED_MAX_HEIGHT = 98  # b
    MOVE_DOWN = 99         # c
    MOVE_UP = 100          # d
    STOP_MOVE = 101        # e
    CALIBRATE = 102        # f
    ROS_MOVE = 103         # g


class LiftNode:

    def __init__(self, port='/dev/ttyLIFT'):
        self.port = port

        rospy.init_node("lift_control")
        self.ser_pub = serial.Serial(self.port, 115200, timeout=0.1)
        
        self.cmd_sub = rospy.Subscriber("lift_cmd", String, self.cmd_callback)
        self.info_pub = rospy.Publisher("lift_info", String, queue_size=1)
        self.cmd_resp_pub = rospy.Publisher("lift_cmd_resp", String, queue_size=1)

        # Useful for icarus_cmds.py to track how much the lift has moved
        self.cur_height = None
        self.goal_height = None

        self.read_thread = threading.Thread(target=self.read_serial_thread, daemon=True)
        self.read_thread.start()
        self.write_thread = threading.Thread(target=self.write_serial_thread, daemon=True)
        self.write_thread.start()

        # Flag to prevent lift from moving if it is detected to be uncalibrated
        self.is_lift_calib = False
        self.optical_sensor_triggered = False

        self.print_info_msg("Starting lift control node...")

        # Read last saved value for height and send to Arduino if they are valid
        time.sleep(2) # Needed to give time for serial threads to initialize
        self.init_lift()
        self.print_info_msg("Lift is initialized and ready to receive messages")

        rospy.spin()


    def init_lift(self):
        last_height = None
        max_height = None
        
        # Read and parse last height from file
        try:
            with open(LIFT_LAST_HEIGHT_SAVE_FILE, "r") as last_height_save_file:
                last_height = int(last_height_save_file.read())
        except Exception as e:
            self.print_info_msg(str(e))
            last_height = None

        # Read and parse max height from file
        try:
            with open(LIFT_MAX_HEIGHT_SAVE_FILE, "r") as max_height_save_file:
                max_height = int(max_height_save_file.read())
        except Exception as e:
            self.print_info_msg(str(e))
            max_height = None
        
        # If height value is invalid, tell robot it is
        if (last_height is None or max_height is None) or (last_height == "" or max_height == ""):
            self.is_lift_calib = False
            self.print_info_msg("No saved height information found. Please call 'lift calib' to calibrate the lift")
            return
        
        # If the saved last height is negative, or max height is 0 or negative, this is an invalid state. 
        if last_height < 0 or max_height <= 0:
            self.is_lift_calib = False
            self.print_info_msg("Saved height information is invalid. Please call 'lift calib' to calibrate the lift")
            return     

        # Detect whether lift controller is informing us of optical sensor trigger
        if self.optical_sensor_triggered:
            # In this case, we ignore the saved last height value. We still try to use the saved max_height, however
            # TODO: Figure out if the max_height should be ignored in this case and if the lift should be calibrated
            if last_height != 0:
                last_height = 0
                # self.print_info_msg("Optical sensor triggered during inititialization but last saved height is not 0. " +
                #     "The saved last height will be overwritten to 0 and the saved max height will still be used. " +
                #     " However, it is advisable to recalibrate the lift using 'lift calib'")
        
        self.print_info_msg(f"{last_height} {max_height}", False)
        self.print_info_msg(f"Using saved heights:\n - Cur height: {last_height}\n" +
            f" - Max height: {max_height}\nSending these to lift controller...")

        # Send validated saved heights to the lift controller
        self.is_lift_calib = True
        self.ser_pub.write(f"{chr(Node2ArduinoCommand.SAVED_LAST_HEIGHT)}{last_height}\n".encode())
        self.ser_pub.write(f"{chr(Node2ArduinoCommand.SAVED_MAX_HEIGHT)}{max_height}\n".encode())


    # Function to read serial output from the lift controller
    def read_serial_thread(self):
        while True:
            s = self.ser_pub.read(1) # Only returns 1 character at a time
            if s == b'':
                continue
            
            cmd = ord(s.decode())
            msg = self.read_serial_till_newline()

            # Lift controller sent us info that a ROS command was completed
            if cmd == Arduino2NodeCommand.ROS_CMD_COMPLETE:
                self.cmd_resp_pub.publish("complete")

            # Lift is sending back the current height
            elif cmd == Arduino2NodeCommand.CUR_HEIGHT:                                
                # Get rid of /r and decimal points at end
                # Also ensure that cur_height does not go below 0,
                # as this causes the lift to not know how to move
                self.cur_height = max(int(float(msg[:-1])), 0)
                
                # Save last height in file (overwrite old height)
                with open(LIFT_LAST_HEIGHT_SAVE_FILE, "w+") as last_height_save_file:
                    last_height_save_file.write(
                        str(self.cur_height) 
                    )

                if self.goal_height is not None:
                    self.cmd_resp_pub.publish(f"{self.cur_height} {self.goal_height}")
            
            # Lift is sending back maximum set height right after calibration
            elif cmd == Arduino2NodeCommand.MAX_HEIGHT:
                # If the lift returns the max height, this can only happen at
                # the end of a calibration. Set calibration status in this case
                self.is_lift_calib = True
                
                # Save max height in file (overwrite old height)
                with open(LIFT_MAX_HEIGHT_SAVE_FILE, "w+") as max_height_save_file:
                    max_height_save_file.write(
                        str(int(float(msg[:-1]))) # Get rid of /r and decimal points at end
                    ) 
                    self.print_info_msg(f"Max height set to {msg}")

            # Lift is sending back a goal height that it is trying to get to
            elif cmd == Arduino2NodeCommand.GOAL_HEIGHT:
                self.goal_height = int(float(msg[:-1])) # Get rid of /r and decimal points at end
                self.print_info_msg(f"Moving to goal {self.goal_height}")

            elif cmd == Arduino2NodeCommand.END_CALIBRATION:
                self.print_info_msg("Calibration ended!")
                self.init_lift()

            # Lift is sending back over current warning/error. 
            elif cmd == Arduino2NodeCommand.CURRENT_LIMIT_TRIGGERED:
                # We don't reset the heights here since the overload could just be from an obstruction
                # at the current height
                self.print_info_msg("CURRENT OVERLOAD DETECTED. EMERGENCY STOP")                
                # Don't tell ROS to complete and move onto the next command in queue, user should address this ASAP
            
            # Current limit warning was reset :)
            elif cmd == Arduino2NodeCommand.CURRENT_LIMIT_UNTRIGGERED:
                self.print_info_msg("No more current overload. However, previous action was skipped")
                self.cmd_resp_pub.publish("complete")

            # If serial output starts with o, this is telling the node that the optical
            # sensor is being triggered
            elif cmd == Arduino2NodeCommand.OPTICAL_SENSOR_BLOCKED:
                self.print_info_msg("Optical sensor triggered")
                self.cmd_resp_pub.publish("complete")
                self.optical_sensor_triggered = True

            # Optical sensor just turned off
            elif cmd == Arduino2NodeCommand.OPTICAL_SENSOR_UNBLOCKED:
                self.print_info_msg("Optical sensor untriggered")
                self.optical_sensor_triggered = False

            elif cmd == Arduino2NodeCommand.OPTICAL_SENSOR_OFF:
                self.print_info_msg("Optical sensor is off. Lift will not respond to commands")
                self.cmd_resp_pub.publish("complete")

            else:
                self.print_info_msg("Other message from lift: " + msg)

            
    # Thread used to send command line input from the user directly to the Arduino
    def write_serial_thread(self):
        while True:
            msg = input()
            msg = msg + "\n"
            self.ser_pub.write(msg.encode())


    # Handles string messages received from ROS to control the lift
    def cmd_callback(self, cmd_string):                
        str_msg = str(cmd_string.data)
        send_str = ""
        
        if str_msg == "calib":
            self.is_lift_calib = False
            send_str = f"{chr(Node2ArduinoCommand.CALIBRATE)}\n"

        elif str_msg == "stop":
            send_str = f"{chr(Node2ArduinoCommand.STOP_MOVE)}\n"

        else:
            # Do not allow the lift to be send any commands to move
            # until it is calibrated
            if self.is_lift_calib is False:
                self.info_pub.publish("Lift uncalibrated, please calibrate it first before proceeding")
                self.cmd_resp_pub.publish("complete")
                return

            goal_pos = float(str(cmd_string.data))
            send_str = chr(Node2ArduinoCommand.ROS_MOVE) + str(goal_pos) + "\n"
        
        self.ser_pub.write(send_str.encode())


    def read_serial_till_newline(self):
        msg = ""
        while True:
            cur_char = self.ser_pub.read(1).decode()
            if cur_char == "\n":
                break
            msg += cur_char
        return msg
    

    # Prints a message to the command line and the lift_resp publisher
    def print_info_msg(self, str_msg, is_info = True):
        rospy.loginfo(str_msg)
        if is_info:
            self.info_pub.publish(f"i{str_msg}")
        else:
            self.info_pub.publish(f"n{str_msg}")


if __name__ == "__main__":
    liftNode = LiftNode()