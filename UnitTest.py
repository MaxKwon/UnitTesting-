#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug 17 00:34:32 2017

@author: maxkwon
"""

import numpy as np
import matplotlib.pyplot as plt # plotting package

#dt in seconds
time_step = .01 #10 milliseconds

#time in seconds
time_sweep = 10

class GearPickup:
    
    def __init__(self):
        
        # Stall Torque in N m
        self.stall_torque = 0.71
        # Stall Current in Amps
        self.stall_current = 134
        # Free Speed in RPM
        self.free_speed = 18730
        # Free Current in Amps
        self.free_current = 0.7
        
        self.G = 20
        
        self.J = .1
        
        # Resistance of the motor
        self.R = 12.0 / self.stall_current
        # Motor velocity constant
        self.Kv = ((self.free_speed / 60.0 * 2.0 * np.pi) / (12.0 - self.R * self.free_current))
        # Torque constant
        self.Kt = self.stall_torque / self.stall_current

    
    def getAcceleration(self, voltage, velocity):
       
       acc = (self.Kt * ((self.G * voltage) - (self.G * self.G * velocity)))/(self.J * self.Kv * self.R)
        
       return acc  
   
    def getConstants(self):
        
        print(self.Kv)
        print(self.Kt)
    

class ControlSimulator:
    
    def __init__(self, GearPickup):
        
        self.gearPickup = GearPickup
        
        self.max_saturation = 12
        self.min_saturation = -12
        
        self.Kp = 50
        self.Ki = 0
        self.Kd = 5
        
    def simulate(self, time_dt, total_time, step):
        
        time = 0
        
        position = 0
        velocity = 0
        acceleration = 0
        
        output_voltage = 0
        
        error = 0
        last_error = 0
        
        positions = []
        times = []
        outputs = []
        
        i = 0
        d =0
        
        while (time <= total_time):
            
            acceleration = self.gearPickup.getAcceleration(output_voltage, velocity)
            
            velocity += acceleration * time_step
            position += velocity * time_step
            
            error = step - position 
            
            i += error
            d = error - last_error
            
            P = self.Kp * error
            I = self.Ki * i
            D = self.Kd * d
            
            output_voltage = P + I + D
            
            if (output_voltage > self.max_saturation):
                output_voltage = self.max_saturation
                
            if (output_voltage < self.min_saturation):
                output_voltage = self.min_saturation
                
            
            positions.append(position)
            #outputs.append(output_voltage)
            times.append(time)
            
            last_error = error
            
            time += time_dt
            
        plt.title("Position")
        plt.plot(times, positions, 'r')
        plt.plot(times, outputs, 'b')
        


    
gearPickup = GearPickup()

gearPickup.getConstants()

simulation = ControlSimulator(gearPickup)

simulation.simulate(time_step, time_sweep, .7)
    

