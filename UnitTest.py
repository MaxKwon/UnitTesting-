#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug 17 00:34:32 2017

@author: maxkwon
"""

import numpy as np
import matplotlib.pyplot as plt # plotting package

#dt in seconds
controller_time_step = .01 #10 milliseconds
sim_time_step = .000001

#time in seconds
total_time = 10

class GearPickup:
    
    def __init__(self):
        
        # Stall Torque in N m
        self.stall_torque = 0.71
        # Stall Current in Amps
        self.stall_current = 134.0
        # Free Speed in RPM
        self.free_speed = 18730.0
        # Free Current in Amps
        self.free_current = 0.7
        #gear ratio
        self.G = 20.0
        #Moment of Inertia
        self.J = .1
        # Resistance of the motor
        self.R = .00895
        # Motor velocity constant
        self.Kv = 164.0
        # Torque constant
        self.Kt = .00529
        
    
    def getAcceleration(self, voltage, velocity):
       
       #acc = (self.Kt * ((self.G * voltage) - (self.G * self.G * velocity)))/(self.J * self.Kv * self.R)
       
       #acc = (.00529 * ((20 * voltage) - (20 * 20 * velocity)))/(.1 * 164 * .00895)
       
       acc = (((-self.G**2 * self.Kt)/(self.J * self.Kv * self.R)) * velocity) + (((self.Kt * self.G)/(self.J * self.Kv * self.R)) * voltage)
        
       return acc  
   
    def getConstants(self):
        
        print("Kv: ", self.Kv)
        print("Kt: ", self.Kt)
    

class ControlSimulator:
    
    def __init__(self, GearPickup):
        
        self.gearPickup = GearPickup
        
        self.max_saturation = 12
        self.min_saturation = -12
        
        self.Kp = 40
        self.Ki = 0
        self.Kd = 0
        
    def controlLoop(self, sim_time_step, controller_time_step, total_time, step):
        
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
            times.append(time)
            
            last_error = error
            
            time += controller_time_step
            
            outputs = self.simulate(controller_time_step, sim_time_step, output_voltage, velocity)
            position += outputs[0] 
            velocity = outputs[1]
            
        plt.title("Position")
        plt.plot(times, positions, 'r')
        
    def simulate(self, total_dt, sim_dt, voltage, velocity_init):
        
        time = 0
        
        acceleration = 0
        velocity = velocity_init
        position = 0
        
        outputs = [0,0]
        
        while (time <= total_dt):
            
            acceleration = self.gearPickup.getAcceleration(voltage, velocity)
            
            velocity += acceleration * sim_dt
            position += velocity * sim_dt
            
            time += sim_dt
            
        outputs = [position, velocity]
            
        return outputs
        
       

gearPickup = GearPickup()

gearPickup.getConstants()

simulation = ControlSimulator(gearPickup)

simulation.controlLoop(sim_time_step, controller_time_step, total_time, .7)
    

