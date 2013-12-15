#!/usr/bin/env python

# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2013 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

"""
Driver for reading data from the PyGame API. Used from Inpyt.py for reading input data.
Hacked to include AI 

You will need to modify the following files as shown below
+++ b/lib/cfclient/ui/main.py   
-        self.joystickReader = JoystickReader()
+        self.joystickReader = JoystickReader(cf=self.cf)


+++ b/lib/cfclient/utils/input.py   
+from cfclient.utils.aicontroller import AiController 

-    def __init__(self, do_device_discovery=True):
+    def __init__(self, do_device_discovery=True, cf=None):

-        self.inputdevice = PyGameReader()
+        self.inputdevice = AiController(cf)

You will also need to map the "exit" button to your controller.  This will server as 
the on/off switch for the AI.

You will also likely have to open the tools->parameters tab in the PC-Client which will load the TOC.  

"""

__author__ = 'Steven Arroyo'
__all__ = ['AiController']

import pygame
from pygame.locals import *

import matplotlib.pyplot as plt
from numpy import arange

import math

import time
import logging

from cflib.crazyflie.log import Log
from cfclient.utils.logconfigreader import LogVariable, LogConfig

logger = logging.getLogger(__name__)

class AiController():
    """Used for reading data from input devices using the PyGame API."""
    def __init__(self,cf):
        self.cf = cf
        self.inputMap = None
        pygame.init()

        # AI variables
        self.timer1 = 0
        self.lastTime = 0

	self.num = 0

	# my variables
	self.yaw_error_prev = 0.0
	self.pitch_error_prev = 0.0
	self.roll_error_prev = 0.0
	
	self.total_error_roll = 0.0
	self.total_error_yaw = 0.0
	self.total_error_pitch = 0.0
	
	self.roll_error_array = []
	self.yaw_error_array = []
	self.pitch_error_array = []
	
	self.time_array = []
	

        # ---AI tuning variables---
        # This is the thrust of the motors duing hover.  0.5 reaches ~1ft depending on battery
        self.maxThrust = 0.48
        # Determines how fast to take off
        self.thrustInc = 0.039
        self.takeoffTime = 0.5
        #self.takeoffTime = 1
        # Determines how fast to land
        self.thrustDec = -0.019
        self.hoverTime = 1
        # Sets the delay between test flights\
        self.repeatDelay = 2

        # parameters pulled from json with defaults from crazyflie pid.h
        # perl -ne '/"(\w*)": {/ && print $1,  "\n" ' lib/cflib/cache/27A2C4BA.json
        self.cfParams = {
            #'pid_rate.pitch_kp': 70.0, 
            'pid_rate.pitch_kd': 0.0, 
            'pid_rate.pitch_ki': 230.0,
	    'pid_rate.pitch_kp':260,  # my input 
            #'pid_rate.roll_kp': 70.0,
	    'pid_rate.roll_kp':260.0, 
            'pid_rate.roll_kd': 0.0, 
            'pid_rate.roll_ki': 230.0, 
            #'pid_rate.yaw_kp': 50.0,
	    'pid_rate.yaw_kp': 25, 
            'pid_rate.yaw_kd': 0.0, 
            'pid_rate.yaw_ki': 25.0, 
            'pid_attitude.pitch_kp': 3.5, 
            'pid_attitude.pitch_kd': 0.0, 
            'pid_attitude.pitch_ki': 2.0, 
            'pid_attitude.roll_kp': 3.5, 
            'pid_attitude.roll_kd': 0.0, 
            'pid_attitude.roll_ki': 2.0, 
            'pid_attitude.yaw_kp': 0.0, 
            'pid_attitude.yaw_kd': 0.0, 
            'pid_attitude.yaw_ki': 0.0,
	    'pid_error.roll_rate_error' : 0.0,
	    'pid_error.pitch_rate_error' : 0.0,
	    'pid_error.yaw_rate_error' : 0.0,
            'sensorfusion6.kp': 0.800000011921, 
            'sensorfusion6.ki': 0.00200000009499, 
            'imu_acc_lpf.factor': 32 }

    def readInput(self):
        """Read input from the selected device."""

        # First we read data from controller as normal
        # ----------------------------------------------------
        # We only want the pitch/roll cal to be "oneshot", don't
        # save this value.
        self.data["pitchcal"] = 0.0
        self.data["rollcal"] = 0.0
        for e in pygame.event.get():
          if e.type == pygame.locals.JOYAXISMOTION:
            index = "Input.AXIS-%d" % e.axis 
            try:
                if (self.inputMap[index]["type"] == "Input.AXIS"):
                    key = self.inputMap[index]["key"]
                    axisvalue = self.j.get_axis(e.axis)
                    # All axis are in the range [-a,+a]
                    axisvalue = axisvalue * self.inputMap[index]["scale"]
                    # The value is now in the correct direction and in the range [-1,1]
                    self.data[key] = axisvalue
            except Exception:
                # Axis not mapped, ignore..
                pass

          if e.type == pygame.locals.JOYBUTTONDOWN:
            index = "Input.BUTTON-%d" % e.button
            try:
                if (self.inputMap[index]["type"] == "Input.BUTTON"):
                    key = self.inputMap[index]["key"]
                    if (key == "estop"):
                        self.data["estop"] = not self.data["estop"]
                    elif (key == "exit"):
                        # self.data["exit"] = True
                        self.data["exit"] = not self.data["exit"]
                        logger.info("Toggling AI %d", self.data["exit"])
                    else: # Generic cal for pitch/roll
                        self.data[key] = self.inputMap[index]["scale"]
            except Exception:
                # Button not mapped, ignore..
                pass
	    
        # Second if AI is enabled overwrite selected data with AI
        # ----------------------------------------------------------
        if self.data["exit"]:
            self.augmentInputWithAi()

        # Return control Data
        return self.data


    # ELEC424 TODO:  Improve this function as needed
    def augmentInputWithAi(self):
        """
        Overrides the throttle input with a controlled takeoff, hover, and land loop.
        You will to adjust the tuning vaiables according to your crazyflie.  
        The max thrust has been set to 0.3 and likely will not fly.  
        I have found that a value  of 0.5 will reach about 1ft off the ground 
        depending on the battery's charge.
        """

        # Keep track of time
        currentTime = time.time()
        timeSinceLastAi = currentTime - self.lastTime
        self.timer1 = self.timer1 + timeSinceLastAi
        self.lastTime = currentTime
        
        # Basic AutoPilot steadly increase thrust, hover, land and repeat
        # -------------------------------------------------------------
        # delay before takeoff 
        if self.timer1 < 0:
            self.total_error_roll = 0.0
            self.total_error_pitch = 0.0
            self.total_error_yaw = 0.0
	    self.roll_error_array = []
	    self.time_array = []
            self.yawParam()
            self.rollParam()
            self.pitchParam()
            thrustDelta = 0
	    #print "Delay before Takeoff"
        # takeoff
        elif self.timer1 < self.takeoffTime :
            thrustDelta = self.thrustInc
	    #print "Takeoff"
        # hold
        elif self.timer1 < self.takeoffTime + self.hoverTime : 
            thrustDelta = 0
            self.goodness()
	    #self.pidTuner()
	    #print "Hold It"
        # land
        elif self.timer1 < 2 * self.takeoffTime + self.hoverTime :
            thrustDelta = self.thrustDec
            #self.plotter(self.time_array, self.roll_error_array)
	    #print "Land Safely"
	    self.file_write(self.num)
	    self.num = self.num + 1
        # repeat
        else:
            self.timer1 = -self.repeatDelay
            thrustDelta = 0
	    #"Repeat"
            # Example Call to pidTuner
	    #self.plotter(self.time_array, self.roll_error_array)
            self.pidTuner()


        self.addThrust( thrustDelta )


        # override Other inputs as needed
        # --------------------------------------------------------------
        # self.data["roll"] = self.aiData["roll"]
        # self.data["pitch"] = self.aiData["pitch"]
        # self.data["yaw"] = self.aiData["yaw"]
        # self.data["pitchcal"] = self.aiData["pitchcal"]
        # self.data["rollcal"] = self.aiData["rollcal"]
        # self.data["estop"] = self.aiData["estop"]
        # self.data["exit"] = self.aiData["exit"]

    def goodness(self):
    	# Update Params
    	self.cf.param.request_param_update("pid_error.roll_rate_error")
    	self.cf.param.request_param_update("pid_error.pitch_rate_error")
    	#self.cf.param.request_param_update("pid_error.yaw_rate_error")
    	
	# Rate Error
	roll_error = self.cfParams['pid_error.roll_rate_error']
	pitch_error = self.cfParams['pid_error.pitch_rate_error']
	# yaw_error = self.cfParams['pid_error.yaw_rate_error']
	
	self.time_array.append(self.timer1)

	self.roll_error_array.append(roll_error)
	self.pitch_error_array.append(pitch_error)
	# self.yaw_error_array.append(yaw_error)
	
	self.total_error_roll = self.total_error_roll + math.fabs(roll_error)
	self.total_error_pitch = self.total_error_pitch + math.fabs(pitch_error)
	#self.total_error_yaw = self.total_error_yaw + math.fabs(yaw_error)
	
	# Check for oscillation
	roll = True
	pitch = True
	#yaw = True
	if((roll_error * self.roll_error_prev) < 0) :	#oscillation
		print "roll oscillation"
		roll = False
	if((pitch_error * self.pitch_error_prev) < 0) :     #oscillation
                print "pitch oscillation"
		pitch = False
	#if((yaw_error * self.yaw_error_prev) < 0) :     #oscillation
                #print "yaw oscillation"
		#yaw = False
	#return roll, pitch, yaw


    def addThrust(self, thrustDelta):
        # Increment thrust
        self.aiData["thrust"] = self.aiData["thrust"] + thrustDelta 
        # Check for max
        if self.aiData["thrust"] > self.maxThrust:
            self.aiData["thrust"] = self.maxThrust
        # check for min 
        elif self.aiData["thrust"] < 0:
            self.aiData["thrust"] = 0
        # overwrite joystick thrust values
        self.data["thrust"] = self.aiData["thrust"]


    # ELEC424 TODO: Implement this function
    def pidTuner(self):
        """ 
        example on how to update crazyflie params
        """
	#self.cf.param.request_param_update("pid_rate.yaw_kp")
	#self.goodness()
	#if (self.cfParams['pid_rate.roll_kp'] > self.cfParams['pid_rate.roll_ki']):
	if (True) :
		self.cfParams['pid_rate.roll_ki'] = self.cfParams['pid_rate.roll_ki'] + 10
		self.cfParams['pid_rate.pitch_kp'] = self.cfParams['pid_rate.pitch_ki'] + 10
		print "Roll ki:"
		print self.cfParams['pid_rate.roll_ki']
	else :
        	self.cfParams['pid_rate.roll_kp'] = self.cfParams['pid_rate.roll_kp'] + 10
		self.cfParams['pid_rate.pitch_kp'] = self.cfParams['pid_rate.pitch_kp'] + 10
		print "Roll kp:"
		print self.cfParams['pid_rate.roll_kp']
        #self.updateCrazyFlieParam('pid_rate.yaw_kp')
	self.updateCrazyFlieParam('pid_rate.roll_kp')
	self.updateCrazyFlieParam('pid_rate.pitch_kp')
	self.updateCrazyFlieParam('pid_rate.roll_ki')
	self.updateCrazyFlieParam('pid_rate.pitch_ki')
	#self.updateCrazyFlieParam('pid_rate.pitch_ki')
	#self.updateCrazyFlieParam('pid_rate.roll_ki')
	#self.updateCrazyFlieParam('pid_rate.yaw_kp')
	#roll, pitch, yaw = self.goodness()
	#if(yaw):
	#	self.cfParams['pid_rate.yaw_kp'] = self.cfParams['pid_rate.yaw_kp'] + 5
	#if(roll):
		#self.cfParams['pid_rate.roll_kp'] = self.cfParams['pid_rate.roll_kp'] + 5
	#if(pitch):
		#self.cfParams['pid_rate.pitch_kp'] = self.cfParams['pid_rate.pitch_kp'] + 5

	#self.updateCrazyFlieParam('pid_rate.yaw_kp')
	#self.updateCrazyFlieParam('pid_rate.roll_kp')
	#self.updateCrazyFlieParam('pid_rate.pitch_kp')


    # update via param.py -> radiodriver.py -> crazyradio.py -> usbRadio )))
    def updateCrazyFlieParam(self, completename ):
        self.cf.param.set_value( unicode(completename), str(self.cfParams[completename]) )

    # 
    def yawParam(self):
	#self.cf.param.add_update_callback('pid_error.roll_rate_error')
	self.cf.param.add_update_callback(
                    "pid_error.yaw_rate_error",
                    self.paramUpdateCallback)
        
    def rollParam(self):
	#self.cf.param.add_update_callback('pid_error.roll_rate_error')
	self.cf.param.add_update_callback(
                    "pid_error.roll_rate_error",
                    self.paramUpdateCallback)
        
    def pitchParam(self):
	#self.cf.param.add_update_callback('pid_error.roll_rate_error')
	self.cf.param.add_update_callback(
                    "pid_error.pitch_rate_error",
                    self.paramUpdateCallback)

    def paramUpdateCallback(self, name, value):
    	 self.cfParams[name] = float(value)
    	    
    def plotter(self, time_array, function_array):
    	 #print function_array
    	 #print len(function_array)
    	 
    	 #print self.hoverTime
    	 #print len(function_array)
    	 
    	 #iteration = float(self.hoverTime) / float(len(function_array))
	 #print iteration  	 
    	 #time = arange(self.takeoffTime, self.hoverTime, iteration)
    	 
    	 #print time
    	 #print len(time)
    	 
    	 plt.figure(1)
    	 plt.subplot(211)
    	 #plt.plot(time_array, function_array, 'bo')
    	 plt.plot(time_array, function_array, 'bo', time_array, function_array, 'k')
    	 plt.show()

    def file_write(self, num):
	file = open(str(num), 'w')
	file.write("Roll KP:")
	file.write(str(self.cfParams['pid_rate.roll_kp']))
	file.write("Roll KI:")
	file.write(str(self.cfParams['pid_rate.roll_ki']))
	file.write("Time array")
	file.write(str(self.time_array))
	file.write("Roll Error:")
	file.write(str(self.roll_error_array))
	file.close()











    def start_input(self, deviceId, inputMap):
        """Initalize the reading and open the device with deviceId and set the mapping for axis/buttons using the
        inputMap"""
        self.data = {"roll":0.0, "pitch":0.0, "yaw":0.0, "thrust":0.0, "pitchcal":0.0, "rollcal":0.0, "estop": False, "exit":False}
        self.aiData = {"roll":0.0, "pitch":0.0, "yaw":0.0, "thrust":0.0, "pitchcal":0.0, "rollcal":0.0, "estop": False, "exit":False}
        self.inputMap = inputMap
	#self.inputMap = {"key":}
        self.j = pygame.joystick.Joystick(deviceId)
        self.j.init()


    def enableRawReading(self, deviceId):
        """Enable reading of raw values (without mapping)"""
        self.j = pygame.joystick.Joystick(deviceId)
        self.j.init()

    def disableRawReading(self):
        """Disable raw reading"""
        # No need to de-init since there's no good support for multiple input devices
        pass

    def readRawValues(self):
        """Read out the raw values from the device"""
        rawaxis = {}
        rawbutton = {}

        for e in pygame.event.get():
            if e.type == pygame.locals.JOYBUTTONDOWN:
                rawbutton[e.button] = 1
            if e.type == pygame.locals.JOYBUTTONUP:
                rawbutton[e.button] = 0
            if e.type == pygame.locals.JOYAXISMOTION:
                rawaxis[e.axis] = self.j.get_axis(e.axis)

        return [rawaxis,rawbutton]

    def getAvailableDevices(self):
        """List all the available devices."""
        dev = []
        pygame.joystick.quit()
        pygame.joystick.init()
        nbrOfInputs = pygame.joystick.get_count()
        for i in range(0,nbrOfInputs):
            j = pygame.joystick.Joystick(i)
            dev.append({"id":i, "name" : j.get_name()})
        return dev

