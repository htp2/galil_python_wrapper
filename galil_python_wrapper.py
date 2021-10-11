#!/usr/bin/env python

# Written by Henry

from __future__ import print_function
from utilities import gclib
import numpy as np
import os
# If you are having issues importing gclib (e.g. On first install of new computer) you may need to configure using
# the following methods (Linux):
# http://www.galilmc.com/sw/pub/all/doc/gclib/html/ubuntu.html
# http://www.galilmc.com/sw/pub/all/doc/gclib/html/python.html

# If you are having issues connecting to the Arduino (e.g. new computer) - try the following (Linux):
# Unplug arduino, open terminal and type 'dmesg -w'
# plug in arduino and get port name (e.g. /dev/ttyACM0)
# In terminal: 'ls -l /dev/ttyACM0'
# If no permissions, use 'sudo chmod 770 /dev/ttyACM0'
# If not in group 'sudo vi /etc/group', find group name & add user to it. Group name is in the prev list (e.g. dialout)
# Restart computer


class RobotMove:
    """This class contains functions that allow for movement of the robot stage and gripper being used in the Sanaria
    mosquito project at JHU. Generally, these functions wrap some of the lower level galil controller commands, adding
    checks and logic where needed. The class also manages communication with an Arduino to control the gripper. By
    default, but optionally, the robot can be limited into certain allowed volumes."""
    def __init__(self, limit_to_allowed_volumes=True):
        self.galil = self.begin_communication_galil()                # On declaration, start comm, set defaults
        self.set_defaults()
        self.arduino = None
        self.gripper_state = None
        self.arduino_connected = False
        self.limit_to_allowed_volumes = limit_to_allowed_volumes
        if limit_to_allowed_volumes:                                 # Optional limitation of robot to allowed volumes
            self.allowed_volumes_filename = 'data/allowed_volumes.npy'
            exists = os.path.isfile(self.allowed_volumes_filename)
            if exists:                                               # If allowed volumes file exists, read in
                self.allowed_volumes = self.read_allowed_volumes(self.allowed_volumes_filename)
            else:                                                    # else, set to empty array
                self.allowed_volumes = np.ndarray(shape=(0, 2, 3))

    def begin_communication_galil(self, ip_address='192.168.1.1'):
        """This initializes communication with the Galil controller and sets defaults"""
        g = gclib.py()                                               # make an instance of galil comm python class
        try:
            g.GOpen(ip_address + ' --direct -s ALL')                 # connect to controller
            print(g.GInfo())
            return g
        except gclib.GclibError as e:
            print('Ending process due to unexpected gclib error:', e)
            quit()

    def begin_communication_arduino(self):
        """This initializes serial communication with an Arduino microcontroller"""
        import serial
        import time
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 38400, timeout=2)  # Create serial port object
            time.sleep(3)                                      # Give time for connection to be established
            self.arduino.write("b")                            # "b" means begin communication
            line_out = self.arduino.readline()
            if line_out:
                self.arduino_connected = True
                print(line_out)                                # Reads back line that communication is complete
                return self.arduino
            else:
                print('Error initializing Arduino')
                return 0

        except serial.SerialException:
            print('Ending process: Unable to connect to Arduino')
            quit()

    def set_defaults(self):
        """Sets default controller gains, speed, acceleration for Galil controller"""
        self.set_controller_gains()  # set default axis motor gains
        self.set_speed()  # set default axis speeds
        self.set_acceleration()  # set default axis accelerations

    def close_communication(self):
        """This closes communication with the Galil controller"""
        self.save_encoder_vals()                                         # saves current encoder values to file
        self.galil.GClose()                                              # Closes connection to controller
        if self.arduino:
            self.arduino.write("e")                                      # "e" means close communication
            print(self.arduino.readline())                               # Prints line about deactivating motor
            self.arduino.close()                                         # closes serial connection
            self.arduino_connected = False
        return 1

    def set_controller_gains(self, kp=(6, 6, 6, 6), ki=(0, 0, 0, 0), kd=(64, 64, 64, 64)):
        """Inputs (kp, ki, kd) are tuples or lists of length 4, corresponding to the  proportional, integrator, and
        derivative constants for axes (A,B,C,D) respectively.  If you do not wish to change a gain use None as the
        list element """
        if len(kp) == 4 & len(ki) == 4 & len(kd) == 4:                   # check input length
            self.galil.GCommand(list_to_galil_input("KP", kp))           # Write P,I,D gains to controller
            self.galil.GCommand(list_to_galil_input("KI", ki))
            self.galil.GCommand(list_to_galil_input("KD", kd))
            return 1

        else:
            print("Error: Enter values for each axis (A,B,C,D")
            return 0

    def get_controller_gains(self):
        kp = gal_out_to_list(self.galil.GCommand("KP ?,?,?,?"))          # Ask controller for axis PID gains
        ki = gal_out_to_list(self.galil.GCommand("KI ?,?,?,?"))
        kd = gal_out_to_list(self.galil.GCommand("KD ?,?,?,?"))

        if len(kp) == 4 & len(ki) == 4 & len(kd) == 4:                   # Check if output is in expected format
            return kp, ki, kd
        else:
            print("Controller error in gains output")
            return 0

    def set_speed(self, axis_speeds=(25000, 25000, 25000, 25000)):
        """Input axis_speeds is tuple or list of length 4, corresponding to the  speed in encoder counts/sec for each
        axis [A,B,C,D]. If you do not wish to change an speed use None as the list element"""
        if len(axis_speeds) == 4:                                        # check input length
            self.galil.GCommand(list_to_galil_input("SP", axis_speeds))  # Write axis speeds to controller
            return 1

        else:
            print("Error: Enter values for each axis (A,B,C,D")
            return 0

    def get_speed(self):
        speed = gal_out_to_list(self.galil.GCommand("SP ?,?,?,?"))   # Ask the controller for currently set speed values
        if len(speed) == 4:                                          # Check if output is in expected format
            return speed
        else:
            print("Controller error in speed output")
            return 0

    def set_acceleration(self, axis_accels=(256000, 256000, 256000, 256000)):
        """Input axis_speeds is tuple or list of length 4, corresponding to the  speed in encoder counts/sec for each
            axis [A,B,C,D]. If you do not wish to change an acceleration use None as the list element"""
        if len(axis_accels) == 4:                                        # check input length
            self.galil.GCommand(list_to_galil_input("AC", axis_accels))  # Write axis accelerations to controller
            return 1

        else:
            print("Error: Enter values for each axis (A,B,C,D")
            return 0

    def get_acceleration(self):
        accel = gal_out_to_list(self.galil.GCommand("AC ?,?,?,?"))   # Ask the controller for currently set accel values
        if len(accel) == 4:                                          # Check if output is in expected format
            return accel
        else:
            print("Controller error in acceleration output")
            return 0

    def move_relative(self, counts_to_move, latch=1):
        """Instruct the robot to move with tuple or list of length 4, corresponding to the number of encoder counts to
        move for each axis [A,B,C,D]. If latch is set, function will only return when movement is completed and will
        check if the final position was reached as commanded. If you do not wish to move an axis use 0 as the list
        element. NOTE: Rotations can not be commanded with this package. Use Galiltools (or edit) if needed"""
        counts_to_move = [0 if c is None else c for c in counts_to_move]  # Adjust for accidental input of None
        counts_to_move[-1] = 0                                            # force all rotations to zero
        if self.limit_to_allowed_volumes:                                 # if limiting movement regions:
            command_position = [p+m for p, m in zip(self.get_position(), counts_to_move)]  # calculate commanded pos
            if self.check_if_allowed(command_position[0:3]):              # check if command is allowed
                pass                                                      # if not allowed, errors in function
        self.galil.GCommand(list_to_galil_input("PR", counts_to_move))    # Set relative movement on each axis
        self.galil.GCommand("BG")                                         # Command movement to begin

        if latch:                                                         # Optional: hold return until move complete
            self.wait_until_stop()
            self.galil.GCommand(list_to_galil_input("PR", [0, 0, 0, 0]))  # zero PR buffer
        return 1

    def move_absolute(self, goal, latch=1):
        """Instruct the robot to move to a specific position with tuple or list of length 4, corresponding to the
            location in encoder counts to move to for each axis [A,B,C,D]. If latch is set, function will only return
             when movement is completed and will check if the final position was reached as commanded. If you do not
              wish to move an axis use None as the list element. NOTE: Rotations can not be commanded with this package.
               Use Galiltools (or edit) if needed"""
        self.galil.GCommand(list_to_galil_input("PR", [0, 0, 0, 0]))   # Ensure only commanded mvmt by zeroing PR buffer
        goal[-1] = None                                                # stop any commanded rotations
        current = self.get_position()
        command = [current[x] if goal[x] is None                       # calculate commanded position
                   else goal[x] for x in xrange(len(goal))]
        if self.limit_to_allowed_volumes:                              # if limiting movement regions:
            if self.check_if_allowed(command[0:3]):                    # check if command is allowed
                pass                                                   # if not allowed, errors in function

        self.galil.GCommand(list_to_galil_input("PA", goal))  # Set locations for movement on each axis
        self.galil.GCommand("BG")                                      # Command movement to begin

        if latch:                                                      # Optional: hold return until move complete
            self.wait_until_stop()
            final_position = self.get_position()
            if final_position != command:                              # Determine if final position is the expected one
                print("Robot did not reach goal position of " + str(command) + ", but is instead at position "
                      + str(final_position))
        return 1

    def wait_until_stop(self, axes="ABCD"):
        """Input axes to wait on as a string of non-separated axis names. e.g. "ABCD", "BC", etc.
            Defaults to wait on all axes"""
        self.galil.GMotionComplete(axes)      # Wait for motion in all axes to complete
        return 1

    def get_position(self):  # outputs a list of encoder counts [A,B,C,D]
        """Asks the galil controller for current encoder values and outputs as a list [x,y,z,r] / ([a,b,c,d])"""
        position = gal_out_to_list(self.galil.GCommand("TP A,B,C,D"))  # Ask controller for position (encoder counts)
        if len(position) == 4:                                         # Check if output is in expected format
            position = [int(i) for i in position]                      # Force to ints
            return position
        else:
            print("Error: Controller error in position output")
            quit()

    def get_velocity(self):
        """Outputs a list of encoder counts/sec [A,B,C,D]. Useful to determine if movement has stopped"""
        velocity = gal_out_to_list(self.galil.GCommand("TV A,B,C,D"))  # Gets current velocity (counts/sec)
        if len(velocity) == 4:                                         # Check if output is in expected format
            return velocity
        else:
            print("Controller error in velocity output")
            return 0

    def open_gripper(self):
        """Send the arduino a serial command to move the servo and open the gripper"""
        if self.arduino_connected:
            self.arduino.write("o")             # "o" means open gripper
            self.arduino.readline()             # Delays function progress until Arduino is finished (or timeout)
            self.gripper_state = 0              # Define gripper state as open ("0")
            return 1
        else:
            print("Error: Connection to gripper Arduino not established. Use 'begin_communication_arduino'")
            return 0

    def close_gripper(self):
        """Send the arduino a serial command to move the servo and close the gripper"""
        if self.arduino_connected:
            self.arduino.write("c")             # "c" means close gripper
            self.arduino.readline()             # Delays function progress until Arduino is finished (or timeout)
            self.gripper_state = 1              # Define gripper state as closed ("1")
            return 1
        else:
            print("Error: Connection to gripper Arduino not established. Use 'begin_communication_arduino'")
            return 0

    def get_gripper_state(self):
        """Tells what state the program believes the gripper to be in (i.e. what was the most recent instruction)"""
        return self.gripper_state

    def home_axis(self, axis):
        """Starts a homing routine on specified axis. Will move the specified axes in the negative direction until the
            limit switch, advance forward to an encoder pulse, and set that position as 0. Specify axes
             with its character name as a string (e.g. "B", "C" or "D") """
        start_speed_setting = self.get_speed()
        self.set_speed((5000, 5000, 5000, 5000))  # Slow the motors down

        self.galil.GCommand("HM " + axis)                                # Home Axis
        self.galil.GCommand("BG " + axis)
        self.wait_until_stop()                                           # Wait for movement to be completed
        self.galil.GSleep(1000)                                          # Wait 1 second to ensure move is done
        self.galil.GCommand("DP" + axis + "=0")                          # Set the D axis "home" position to 0
        self.set_speed(start_speed_setting)                              # Return motors to previous speed

    def set_position(self, positions):
        """Input new position values for current position with tuple or list of length 4 that defines position for each
            axis [A,B,C,D]. If you do not wish to change the value of an axis, set list element to None"""
        if len(positions) == 4:                                          # check input length
            self.galil.GCommand(list_to_galil_input("DP", positions))    # Write new position values to controller
            return 1
        else:
            print("Error: Enter values for each axis (A,B,C,D)")
            return 0

    def read_allowed_volumes(self, filename):
        """Read the allowed volumes from local file (typically used during init). If no file found, initialize allowed
        volumes list as empty"""
        try:
            return np.load(filename)                                     # read array from file if exists
        except IOError:
            print('No file :', filename, ' found to import allowed regions from. Add allowed regions using add_allowed_'
                                         'volume')
            return np.ndarray(shape=(0, 2, 3))                           # otherwise make empty array

    def add_allowed_volume(self, low_bound, high_bound):
        """Input is two lists low_bound and high_bound which are python lists with 3 elements [x,y,z] corresponding to
        the values of points at the corner of a 3D volume in which the robot is allowed to move. The values correspond
        to the x,y,z (or a,b,c) axes respectively. The values of all elements in low_bound must be strictly less than
        those in high_bound. All values are in encoder counts of the robot"""
        lb = np.array(low_bound)  # make numpy arrays
        hb = np.array(high_bound)
        if (lb < hb).all():
            new_bound = np.array([[lb, hb]])                             # make new bound object
            if self.allowed_volumes.any():                               # if exists, append to current array
                self.allowed_volumes = np.append(self.allowed_volumes, new_bound, axis=0)
            else:                                                        # else fill new array
                self.allowed_volumes = new_bound
            np.save(self.allowed_volumes_filename, self.allowed_volumes)
            print('Allowed volume added')
            return 1
        else:
            print('Error in bound declaration. low_bound should be smaller than high_bound.')
            quit()

    def remove_allowed_volume(self, remove_indices, clear_all=False):
        """Removes one or more allowed volumes for the system, both in the local class variables and in the file from
        which allowed volumes are read in at startup. List the volume indices in a list [i,j,k]. Try using
        'list_allowed_volumes' to see which ones to remove. Set clear_all flag to true to completely reset volumes."""
        if clear_all:  # Resets all
            self.allowed_volumes = np.ndarray(shape=(0, 2, 3))
        else:
            mask = np.ones(len(self.allowed_volumes), dtype=bool)
            mask[remove_indices] = False
            self.allowed_volumes = self.allowed_volumes[mask, ...]
        np.save(self.allowed_volumes_filename, self.allowed_volumes)
        print('Allowed volume removed')
        return 1

    def list_allowed_volumes(self):
        """Prints a viewing-friendly list of all allowed volumes to the screen. Can be used along with
        'remove_allowed_volume' to find candidate volumes for removal"""
        if self.allowed_volumes.any():  # if exists,
            for volume in xrange(np.shape(self.allowed_volumes)[0]):
                print('Volume: ', volume)
                print('Low Bound:', list(self.allowed_volumes[volume][0]))
                print('High Bound:', list(self.allowed_volumes[volume][1]))
        else:
            print('No allowed volumes have been specified. Consider using add_allowed_volumes')
        return 1

    def check_if_allowed(self, command):
        """Input is a commanded position list in [x,y,z] or [a,b,c] coordinates (rotational moves not allowed
        currently), check to see if that position would lie in an allowed region. If so, return, else quit."""
        command = np.array(command)
        if self.allowed_volumes.any():  # if exists,
            for volume in xrange(np.shape(self.allowed_volumes)[0]):
                low_bound = self.allowed_volumes[volume][0]
                high_bound = self.allowed_volumes[volume][1]
                if ((command > low_bound).all()) and ((command < high_bound).all()):
                    return 1
        else:
            print('No allowed volumes have been specified. Consider using add_allowed_volumes')
        print('Error: Attempted move outside of allowed volumes.')
        quit()

    def save_encoder_vals(self):
        """Saves current encoder values to file. When run at shutdown prevents re-homing in event of controller
            losing power"""
        save_file = open('data/encoder_at_shutdown.txt', 'w')        # Save current encoder values to file
        save_file.write(str(self.get_position()))
        save_file.close()
        # Also could be done using galiltools, just use "TP x,y,z,a"

    def reset_encoders_from_file(self):
        """Resets encoder values from the file saved in save_encoder_values. To be used in event of controller
        loss of power"""
        try:
            load_file = open('data/encoder_at_shutdown.txt', 'r')
            import ast
            last_position = ast.literal_eval(load_file.readline())
            last_position = [int(i) for i in last_position]
            self.set_position(last_position)
        except:
            print('Error: No file with previous encoder readings found or error parsing file')
            quit()


# Helper Functions
def gal_out_to_list(galil_output):
    """Converts output string for Galil controller to Python list"""
    list_output = []
    for s in galil_output.split(","):                                # Split axis values out of comma-separated string
        if s.isdigit():
            list_output.append(int(s))                               # If integer (e.g. motor counts), append as such
        else:
            list_output.append(float(s))                             # If float (e.g. gains), append as such
    return list_output


def list_to_galil_input(galil_command_str, list_input):
    """Turn list into galil controller-friendly input"""
    list_input_str = []
    for s in list_input:
        if s is None:
            list_input_str.append("")
        else:
            list_input_str.append(str(s))
    galil_input = galil_command_str + " " + ",".join(list_input_str)
    return galil_input
