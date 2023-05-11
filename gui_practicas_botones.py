#!/usr/bin/env python3
# encoding:utf-8


from tkinter import *
#import ttk
from tkinter.ttk import *
from tkinter import ttk
import tkinter.font as font

# HAY QUE INSTALAR EL PAQUETE ImageTk con: apt-get install python3-pil.imagetk
from PIL import ImageTk, Image

import os, time
import cv2
#from cv2 import cv2

import threading
from multiprocessing import Process

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import math
from math import pi, cos, sin
import numpy as np

# HAY QUE INSTALAR EL PAQUETE pyperclip con: apt-get install python3-pyperclip
import pyperclip

IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"

print(cv2.__version__)

class MoveGroupPythonIntefaceTutorial(object):
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    group_name = "lite6"
    group = moveit_commander.MoveGroupCommander(group_name)
    
    #self.tipo_grupo()
    self.group = group

  def plan_cartesian_path_x(self, scale=0.001):
    
    desplazamiento_float = desplazamiento.get()
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.x += scale * float(desplazamiento_float)
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.execute(plan, wait=True)
    return plan, fraction

  def plan_cartesian_path_y(self, scale=0.001):
    
    desplazamiento_float = desplazamiento.get()
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.y += scale * float(desplazamiento_float)
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.execute(plan, wait=True)
    return plan, fraction

  def plan_cartesian_path_z(self, scale=0.001):
    
    desplazamiento_float = desplazamiento.get()
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.z += scale * float(desplazamiento_float)
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.execute(plan, wait=True)
    return plan, fraction

  def plan_cartesian_path_x_10(self, scale=1):
    
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.x += scale * 0.01
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.execute(plan, wait=True)
    return plan, fraction  

  def plan_cartesian_path_y_10(self, scale=1):
    
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.y += scale * 0.01
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.execute(plan, wait=True)
    return plan, fraction

  def plan_cartesian_path_z_10(self, scale=1):
    
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.z += scale * 0.01
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.execute(plan, wait=True)
    return plan, fraction

  def plan_cartesian_path_neg_x(self, scale=1):
    
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.x -= scale * 0.01
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.execute(plan, wait=True)
    return plan, fraction

  def plan_cartesian_path_neg_y(self, scale=1):
    
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.y -= scale * 0.01
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.execute(plan, wait=True)
    return plan, fraction

  def plan_cartesian_path_neg_z(self, scale=1):
    
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.z -= scale * 0.01
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.execute(plan, wait=True)
    return plan, fraction

  def plan_cartesian_path_x_100(self, scale=1):
    
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.x += scale * 0.1
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.execute(plan, wait=True)
    return plan, fraction

  def plan_cartesian_path_y_100(self, scale=1):
    
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.y += scale * 0.1
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.execute(plan, wait=True)
    return plan, fraction

  def plan_cartesian_path_z_100(self, scale=1):
    
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.z += scale * 0.1
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.execute(plan, wait=True)
    return plan, fraction

  def plan_cartesian_path_neg_x_100(self, scale=1):
    
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.x -= scale * 0.1
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.execute(plan, wait=True)
    return plan, fraction

  def plan_cartesian_path_neg_y_100(self, scale=1):
    
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.y -= scale * 0.1
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.execute(plan, wait=True)
    return plan, fraction

  def plan_cartesian_path_neg_z_100(self, scale=1):
    
    group = self.group
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.z -= scale * 0.1
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.execute(plan, wait=True)
    return plan, fraction

  def mueve_coord2(self):
        coord_x_float = coord_x.get()
        coord_y_float = coord_y.get()
        coord_z_float = coord_z.get()
        group = self.group
        x0 = group.get_current_pose().pose.position.x
        y0 = group.get_current_pose().pose.position.y
        z0 = group.get_current_pose().pose.position.z
        w_orientation = group.get_current_pose().pose.orientation.w
        x_orientation = group.get_current_pose().pose.orientation.x
        y_orientation = group.get_current_pose().pose.orientation.y
        z_orientation = group.get_current_pose().pose.orientation.z
        
        x0 = float(coord_x_float)/1000 if (coord_x_float)!="" else x0
        y0 = float(coord_y_float)/1000 if (coord_y_float)!="" else y0
        z0 = float(coord_z_float)/1000 if (coord_z_float)!="" else z0
    
        waypoints = []
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = w_orientation
        pose_goal.orientation.x = x_orientation
        pose_goal.orientation.y = y_orientation
        pose_goal.orientation.z = z_orientation
        pose_goal.position.x = x0
        pose_goal.position.y = y0
        pose_goal.position.z = z0
        group.set_pose_target(pose_goal)
        poses = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
        waypoints.append(copy.deepcopy(poses))
        plan = group.go(wait=True)

  def copia_posicion2(self):
        group = self.group
        x0 = group.get_current_pose().pose.position.x
        y0 = group.get_current_pose().pose.position.y
        z0 = group.get_current_pose().pose.position.z
        w_orientation = group.get_current_pose().pose.orientation.w
        x_orientation = group.get_current_pose().pose.orientation.x
        y_orientation = group.get_current_pose().pose.orientation.y
        z_orientation = group.get_current_pose().pose.orientation.z
        
        pyperclip.copy("pose_goal.orientation.w = "+str(w_orientation)+
          "\npose_goal.orientation.x = "+str(x_orientation)+
          "\npose_goal.orientation.y = "+str(y_orientation)+
          "\npose_goal.orientation.z = "+str(z_orientation)+
          "\npose_goal.position.x = "+str(x0)+
          "\npose_goal.position.y = "+str(y0)+
          "\npose_goal.position.z = "+str(z0)+
          "\n"+"\n")
        time.sleep(0.5)
        copy = pyperclip.paste()
        #posicion = copy.get()
        copy = posicion.insert(END, copy)
        return copy

  def copia_angulos2(self):
        group = self.group
        joint_goal = group.get_current_joint_values()
        angulo1 = joint_goal[0]
        angulo2 = joint_goal[1]
        angulo3 = joint_goal[2]
        angulo4 = joint_goal[3]
        angulo5 = joint_goal[4]
        angulo6 = joint_goal[5]
        
        pyperclip.copy("joint_goal[0] = "+str(angulo1)+
          "\njoint_goal[1] = "+str(angulo2)+
          "\njoint_goal[2] = "+str(angulo3)+
          "\njoint_goal[3] = "+str(angulo4)+
          "\njoint_goal[4] = "+str(angulo5)+
          "\njoint_goal[5] = "+str(angulo6)+
          "\n"+"\n")
        time.sleep(0.5)
        copy = pyperclip.paste()
        #posicion = copy.get()
        copy = posicion.insert(END, copy)
        return copy

  def mueve_15g_beta(self):
        group = self.group
        x0 = group.get_current_pose().pose.position.x
        y0 = group.get_current_pose().pose.position.y
        z0 = group.get_current_pose().pose.position.z
        w_orientation = group.get_current_pose().pose.orientation.w
        x_orientation = group.get_current_pose().pose.orientation.x
        y_orientation = group.get_current_pose().pose.orientation.y
        z_orientation = group.get_current_pose().pose.orientation.z

        euler = self.get_euler_from_quaternion(x_orientation, y_orientation, z_orientation, w_orientation)
    
        #alpha = float(input("Elige el angulo de Euler alpha alrededor del eje X en grados sexagesimales: "))
        alpha = euler[0]#+(alpha*pi/180)
        #beta = float(input("Elige el angulo de Euler beta alrededor del eje Y en grados sexagesimales: "))
        beta = euler[1]+(15*pi/180)
        #gamma = float(input("Elige el angulo de Euler gamma alrededor del eje Z en grados sexagesimales: "))
        gamma = euler[2]#+(gamma*pi/180)

        quater = self.get_quaternion_from_euler(alpha, beta, gamma)
        print ("quater")
        print (quater)
    
        waypoints = []
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = quater[3]
        pose_goal.orientation.x = quater[0]
        pose_goal.orientation.y = quater[1]
        pose_goal.orientation.z = quater[2]
        pose_goal.position.x = x0
        pose_goal.position.y = y0
        pose_goal.position.z = z0
        group.set_pose_target(pose_goal)
        poses = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
        waypoints.append(copy.deepcopy(poses))
        plan = group.go(wait=True)

  def mueve_15g_alpha(self):
        group = self.group
        x0 = group.get_current_pose().pose.position.x
        y0 = group.get_current_pose().pose.position.y
        z0 = group.get_current_pose().pose.position.z
        w_orientation = group.get_current_pose().pose.orientation.w
        x_orientation = group.get_current_pose().pose.orientation.x
        y_orientation = group.get_current_pose().pose.orientation.y
        z_orientation = group.get_current_pose().pose.orientation.z

        euler = self.get_euler_from_quaternion(x_orientation, y_orientation, z_orientation, w_orientation)
    
        #alpha = float(input("Elige el angulo de Euler alpha alrededor del eje X en grados sexagesimales: "))
        alpha = euler[0]+(15*pi/180)
        #beta = float(input("Elige el angulo de Euler beta alrededor del eje Y en grados sexagesimales: "))
        beta = euler[1]#+(15*pi/180)
        #gamma = float(input("Elige el angulo de Euler gamma alrededor del eje Z en grados sexagesimales: "))
        gamma = euler[2]#+(gamma*pi/180)

        quater = self.get_quaternion_from_euler(alpha, beta, gamma)
        print ("quater")
        print (quater)
    
        waypoints = []
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = quater[3]
        pose_goal.orientation.x = quater[0]
        pose_goal.orientation.y = quater[1]
        pose_goal.orientation.z = quater[2]
        pose_goal.position.x = x0
        pose_goal.position.y = y0
        pose_goal.position.z = z0
        group.set_pose_target(pose_goal)
        poses = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
        waypoints.append(copy.deepcopy(poses))
        plan = group.go(wait=True)

  def mueve_15g_gamma(self):
        group = self.group
        x0 = group.get_current_pose().pose.position.x
        y0 = group.get_current_pose().pose.position.y
        z0 = group.get_current_pose().pose.position.z
        w_orientation = group.get_current_pose().pose.orientation.w
        x_orientation = group.get_current_pose().pose.orientation.x
        y_orientation = group.get_current_pose().pose.orientation.y
        z_orientation = group.get_current_pose().pose.orientation.z

        euler = self.get_euler_from_quaternion(x_orientation, y_orientation, z_orientation, w_orientation)
    
        #alpha = float(input("Elige el angulo de Euler alpha alrededor del eje X en grados sexagesimales: "))
        alpha = euler[0]#+(alpha*pi/180)
        #beta = float(input("Elige el angulo de Euler beta alrededor del eje Y en grados sexagesimales: "))
        beta = euler[1]#+(15*pi/180)
        #gamma = float(input("Elige el angulo de Euler gamma alrededor del eje Z en grados sexagesimales: "))
        gamma = euler[2]+(15*pi/180)

        quater = self.get_quaternion_from_euler(alpha, beta, gamma)
        print ("quater")
        print (quater)
    
        waypoints = []
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = quater[3]
        pose_goal.orientation.x = quater[0]
        pose_goal.orientation.y = quater[1]
        pose_goal.orientation.z = quater[2]
        pose_goal.position.x = x0
        pose_goal.position.y = y0
        pose_goal.position.z = z0
        group.set_pose_target(pose_goal)
        poses = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
        waypoints.append(copy.deepcopy(poses))
        plan = group.go(wait=True)

  def mueve_15g_neg_beta(self):
        group = self.group
        x0 = group.get_current_pose().pose.position.x
        y0 = group.get_current_pose().pose.position.y
        z0 = group.get_current_pose().pose.position.z
        w_orientation = group.get_current_pose().pose.orientation.w
        x_orientation = group.get_current_pose().pose.orientation.x
        y_orientation = group.get_current_pose().pose.orientation.y
        z_orientation = group.get_current_pose().pose.orientation.z

        euler = self.get_euler_from_quaternion(x_orientation, y_orientation, z_orientation, w_orientation)
    
        #alpha = float(input("Elige el angulo de Euler alpha alrededor del eje X en grados sexagesimales: "))
        alpha = euler[0]#+(alpha*pi/180)
        #beta = float(input("Elige el angulo de Euler beta alrededor del eje Y en grados sexagesimales: "))
        beta = euler[1]-(15*pi/180)
        #gamma = float(input("Elige el angulo de Euler gamma alrededor del eje Z en grados sexagesimales: "))
        gamma = euler[2]#+(15*pi/180)

        quater = self.get_quaternion_from_euler(alpha, beta, gamma)
        print ("quater")
        print (quater)
    
        waypoints = []
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = quater[3]
        pose_goal.orientation.x = quater[0]
        pose_goal.orientation.y = quater[1]
        pose_goal.orientation.z = quater[2]
        pose_goal.position.x = x0
        pose_goal.position.y = y0
        pose_goal.position.z = z0
        group.set_pose_target(pose_goal)
        poses = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
        waypoints.append(copy.deepcopy(poses))
        plan = group.go(wait=True)

  def mueve_15g_neg_alpha(self):
        group = self.group
        x0 = group.get_current_pose().pose.position.x
        y0 = group.get_current_pose().pose.position.y
        z0 = group.get_current_pose().pose.position.z
        w_orientation = group.get_current_pose().pose.orientation.w
        x_orientation = group.get_current_pose().pose.orientation.x
        y_orientation = group.get_current_pose().pose.orientation.y
        z_orientation = group.get_current_pose().pose.orientation.z

        euler = self.get_euler_from_quaternion(x_orientation, y_orientation, z_orientation, w_orientation)
    
        #alpha = float(input("Elige el angulo de Euler alpha alrededor del eje X en grados sexagesimales: "))
        alpha = euler[0]-(15*pi/180)
        #beta = float(input("Elige el angulo de Euler beta alrededor del eje Y en grados sexagesimales: "))
        beta = euler[1]#-(15*pi/180)
        #gamma = float(input("Elige el angulo de Euler gamma alrededor del eje Z en grados sexagesimales: "))
        gamma = euler[2]#+(15*pi/180)

        quater = self.get_quaternion_from_euler(alpha, beta, gamma)
        print ("quater")
        print (quater)
    
        waypoints = []
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = quater[3]
        pose_goal.orientation.x = quater[0]
        pose_goal.orientation.y = quater[1]
        pose_goal.orientation.z = quater[2]
        pose_goal.position.x = x0
        pose_goal.position.y = y0
        pose_goal.position.z = z0
        group.set_pose_target(pose_goal)
        poses = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
        waypoints.append(copy.deepcopy(poses))
        plan = group.go(wait=True)

  def mueve_15g_neg_gamma(self):
        group = self.group
        x0 = group.get_current_pose().pose.position.x
        y0 = group.get_current_pose().pose.position.y
        z0 = group.get_current_pose().pose.position.z
        w_orientation = group.get_current_pose().pose.orientation.w
        x_orientation = group.get_current_pose().pose.orientation.x
        y_orientation = group.get_current_pose().pose.orientation.y
        z_orientation = group.get_current_pose().pose.orientation.z

        euler = self.get_euler_from_quaternion(x_orientation, y_orientation, z_orientation, w_orientation)
    
        #alpha = float(input("Elige el angulo de Euler alpha alrededor del eje X en grados sexagesimales: "))
        alpha = euler[0]#-(15*pi/180)
        #beta = float(input("Elige el angulo de Euler beta alrededor del eje Y en grados sexagesimales: "))
        beta = euler[1]#-(15*pi/180)
        #gamma = float(input("Elige el angulo de Euler gamma alrededor del eje Z en grados sexagesimales: "))
        gamma = euler[2]-(15*pi/180)

        quater = self.get_quaternion_from_euler(alpha, beta, gamma)
        print ("quater")
        print (quater)
    
        waypoints = []
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = quater[3]
        pose_goal.orientation.x = quater[0]
        pose_goal.orientation.y = quater[1]
        pose_goal.orientation.z = quater[2]
        pose_goal.position.x = x0
        pose_goal.position.y = y0
        pose_goal.position.z = z0
        group.set_pose_target(pose_goal)
        poses = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
        waypoints.append(copy.deepcopy(poses))
        plan = group.go(wait=True)

  def mueve_beta(self):
        rotacion_float = rotacion.get()
        group = self.group
        x0 = group.get_current_pose().pose.position.x
        y0 = group.get_current_pose().pose.position.y
        z0 = group.get_current_pose().pose.position.z
        w_orientation = group.get_current_pose().pose.orientation.w
        x_orientation = group.get_current_pose().pose.orientation.x
        y_orientation = group.get_current_pose().pose.orientation.y
        z_orientation = group.get_current_pose().pose.orientation.z

        euler = self.get_euler_from_quaternion(x_orientation, y_orientation, z_orientation, w_orientation)
        print(euler)
        #alpha = float(input("Elige el angulo de Euler alpha alrededor del eje X en grados sexagesimales: "))
        alpha = euler[0]#+(alpha*pi/180)
        #beta = float(input("Elige el angulo de Euler beta alrededor del eje Y en grados sexagesimales: "))
        beta = euler[1]+(float(rotacion_float)*pi/180)
        #gamma = float(input("Elige el angulo de Euler gamma alrededor del eje Z en grados sexagesimales: "))
        gamma = euler[2]#+(gamma*pi/180)

        quater = self.get_quaternion_from_euler(alpha, beta, gamma)
        print ("quater")
        print (quater)
    
        waypoints = []
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = quater[3]
        pose_goal.orientation.x = quater[0]
        pose_goal.orientation.y = quater[1]
        pose_goal.orientation.z = quater[2]
        pose_goal.position.x = x0
        pose_goal.position.y = y0
        pose_goal.position.z = z0
        group.set_pose_target(pose_goal)
        poses = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
        waypoints.append(copy.deepcopy(poses))
        plan = group.go(wait=True)

  def mueve_alpha(self):
        rotacion_float = rotacion.get()
        group = self.group
        x0 = group.get_current_pose().pose.position.x
        y0 = group.get_current_pose().pose.position.y
        z0 = group.get_current_pose().pose.position.z
        w_orientation = group.get_current_pose().pose.orientation.w
        x_orientation = group.get_current_pose().pose.orientation.x
        y_orientation = group.get_current_pose().pose.orientation.y
        z_orientation = group.get_current_pose().pose.orientation.z

        euler = self.get_euler_from_quaternion(x_orientation, y_orientation, z_orientation, w_orientation)
        print(euler)
        #alpha = float(input("Elige el angulo de Euler alpha alrededor del eje X en grados sexagesimales: "))
        alpha = euler[0]+(float(rotacion_float)*pi/180)
        #beta = float(input("Elige el angulo de Euler beta alrededor del eje Y en grados sexagesimales: "))
        beta = euler[1]#+(15*pi/180)
        #gamma = float(input("Elige el angulo de Euler gamma alrededor del eje Z en grados sexagesimales: "))
        gamma = euler[2]#+(gamma*pi/180)

        quater = self.get_quaternion_from_euler(alpha, beta, gamma)
        print ("quater")
        print (quater)
    
        waypoints = []
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = quater[3]
        pose_goal.orientation.x = quater[0]
        pose_goal.orientation.y = quater[1]
        pose_goal.orientation.z = quater[2]
        pose_goal.position.x = x0
        pose_goal.position.y = y0
        pose_goal.position.z = z0
        group.set_pose_target(pose_goal)
        poses = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
        waypoints.append(copy.deepcopy(poses))
        plan = group.go(wait=True)

  def mueve_gamma(self):
        rotacion_float = rotacion.get()
        group = self.group
        x0 = group.get_current_pose().pose.position.x
        y0 = group.get_current_pose().pose.position.y
        z0 = group.get_current_pose().pose.position.z
        w_orientation = group.get_current_pose().pose.orientation.w
        x_orientation = group.get_current_pose().pose.orientation.x
        y_orientation = group.get_current_pose().pose.orientation.y
        z_orientation = group.get_current_pose().pose.orientation.z

        euler = self.get_euler_from_quaternion(x_orientation, y_orientation, z_orientation, w_orientation)
        print(euler)
        #alpha = float(input("Elige el angulo de Euler alpha alrededor del eje X en grados sexagesimales: "))
        alpha = euler[0]#+(alpha*pi/180)
        #beta = float(input("Elige el angulo de Euler beta alrededor del eje Y en grados sexagesimales: "))
        beta = euler[1]#+(15*pi/180)
        #gamma = float(input("Elige el angulo de Euler gamma alrededor del eje Z en grados sexagesimales: "))
        gamma = euler[2]+(float(rotacion_float)*pi/180)

        quater = self.get_quaternion_from_euler(alpha, beta, gamma)
        print ("quater")
        print (quater)
    
        waypoints = []
        pose_goal = group.get_current_pose().pose
        pose_goal.orientation.w = quater[3]
        pose_goal.orientation.x = quater[0]
        pose_goal.orientation.y = quater[1]
        pose_goal.orientation.z = quater[2]
        pose_goal.position.x = x0
        pose_goal.position.y = y0
        pose_goal.position.z = z0
        group.set_pose_target(pose_goal)
        poses = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
        waypoints.append(copy.deepcopy(poses))
        plan = group.go(wait=True)

  def get_euler_from_quaternion(self, x_orientation, y_orientation, z_orientation, w_orientation):
        """
        Convert a quaternion into euler angles (alpha, beta, gamma)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
   
        Output
          :return alpha, beta, gamma: The orientation in Euler angles [alpha, beta, gamma] format
        """
        t0 = +2.0 * (w_orientation * x_orientation + y_orientation * z_orientation)
        t1 = +1.0 - 2.0 * (x_orientation * x_orientation + y_orientation * y_orientation)
        alpha = math.atan2(t0, t1)
     
        t2 = +2.0 * (w_orientation * y_orientation - z_orientation * x_orientation)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        beta = math.asin(t2)
        print("beta")
        print(beta)
     
        t3 = +2.0 * (w_orientation * z_orientation + x_orientation * y_orientation)
        t4 = +1.0 - 2.0 * (y_orientation * y_orientation + z_orientation * z_orientation)
        gamma = math.atan2(t3, t4)
    
        return [alpha, beta, gamma]

  def get_quaternion_from_euler(self, alpha, beta, gamma):
        """
        Convert an Euler angle to a quaternion.
   
        Input
          :param roll: The roll (rotation around x-axis) angle in radians.
          :param pitch: The pitch (rotation around y-axis) angle in radians.
          :param yaw: The yaw (rotation around z-axis) angle in radians.
 
        Output
          :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(alpha/2) * np.cos(beta/2) * np.cos(gamma/2) - np.cos(alpha/2) * np.sin(beta/2) * np.sin(gamma/2)
        qy = np.cos(alpha/2) * np.sin(beta/2) * np.cos(gamma/2) + np.sin(alpha/2) * np.cos(beta/2) * np.sin(gamma/2)
        qz = np.cos(alpha/2) * np.cos(beta/2) * np.sin(gamma/2) - np.sin(alpha/2) * np.sin(beta/2) * np.cos(gamma/2)
        qw = np.cos(alpha/2) * np.cos(beta/2) * np.cos(gamma/2) + np.sin(alpha/2) * np.sin(beta/2) * np.sin(gamma/2)
 
        return [qx, qy, qz, qw]


class Application(object):
    def __init__(self):
        self.win = Tk()
        # ventana en la parte superior
        self.win.wm_attributes('-topmost', 0)
        self.ros = False
        # archivo para ejecutar
        self.run_py = ""
        # Crear un objeto de tkFont en myFont
        self.myFont = font.Font(family='helvetica')
        self.myFont = font.Font(size=12)
        # Crear y posicionar un recuadro donde poner la imagen



        # establecer título
        self.win.title("LEIAL Technologies - Herramienta de Ejercicios y Simulaciones")
        # Asignamos una imagen para el icono de la ventana
        
        self.win.geometry(
            "825x950+1085+10")  # 830 380 para el tamaño de la ventana，+1150 +10 Defina la ubicación predeterminada cuando aparezca la ventana
        self.win['background']='#006FB9'

        # botón abrir roscore
        #self.btn = Button(self.win, text="Lanzar roscore", command=self.roscore)
        #self.btn.grid(row=0, column=3)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'
        #self.btn['font']='myFont'

        #self.style = ttk.Style()
        #self.style.configure("bg.frame1", background="#006FB9")
        self.frame1 = Frame(self.win, width=845, height=250)
        self.frame1.grid(row=0, column=0, padx=5, pady=5)
        #self.frame1['background']='#006FB9'

        # botón abrir nueva terminal
        self.btn = Button(self.frame1, text="Abrir nueva Terminal", command=self.terminal)
        self.btn.grid(row=1, column=3)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'

        self.chanse_code = Label(self.frame1, text="Elige un programa:", width=15)
        self.chanse_code.grid(row=0, column=0)
        self.chanse_code['background']='#006FB9'
        self.chanse_code['foreground']='#FFFFFF'

        # botón abrir nueva terminal
        self.btn = Button(self.frame1, text="Mycobot Deslizadores", command=self.rviz_desliza_fija)
        self.btn.grid(row=1, column=0)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'

        # botón abrir reconocimiento de color
        self.btn = Button(self.frame1, text="Mycobot MoveIt", command=self.rviz_moveit_fija)
        self.btn.grid(row=2, column=0)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'
        
        self.myComboList = [
                            
                            u"Mycobot Evitar Obstaculos", u"Lite6 Evitar Obstaculos", u"Moveit Commander Command Line", u"Lite6 Robot Practicar soldaduras"
                            ]
        self.myCombox = ttk.Combobox(self.frame1, values=self.myComboList, width=28)
        self.myCombox.grid(row=0, column=2)
        
        self.myComboList2 = [u"Mycobot Deslizadores Pinza Fija", u"Mycobot Deslizadores Pinza Movil", u"Mycobot MoveIt Pinza Fija", u"Mycobot MoveIt Apagado Pinza Fija",
                             u"Mycobot MoveIt Pinza Movil", u"Mycobot Verter Cerveza", u"Mycobot Evitar Obstaculo", u"Mycobot 2 Robots y cinta",
                             u"Mycobot Visión Artificial Moveit", u"Lite6 Moveit pinza fija", u"Lite6 Evitar Obstaculo", u"Lite6 Moveit pinza puntos",
                             u"Lite6 Moveit pinza laser", u"Lite6 Moveit pinza MIG/MAG", u"Lite6 Moveit pinza TIG", u"Lite6 Moveit pinza objeto"]
        self.myCombox2 = ttk.Combobox(self.frame1, values=self.myComboList2, width=28)
        self.myCombox2.grid(row=0, column=1)

        #self.add_btn = Button(self.win, text="Agregar imagen de objeto", command=self.add_img)
        #self.add_btn.grid(row=1, column=2)

        # self.set_xy = Label(self.win, text="set_xy:", width=10)
        # self.set_xy.grid(row=1)

        # self.x = Label(self.win, text="x:")
        # self.x.grid(row=2)
        # self.v1 = StringVar()
        # self.e1 = Entry(self.win,textvariable=self.v1, width=10)
        # self.e1.insert(0,0)
        # self.e1.grid(row=2,column=1)

        # self.y = Label(self.win, text="y:")
        # self.y.grid(row=3)
        # self.v2 = StringVar()
        # self.e2 = Entry(self.win,textvariable=self.v2, width=10)
        # self.e2.insert(0,0)
        # self.e2.grid(row=3,column=1)
        self.tips = "1. Seleccionar antes en el primer desplegable a la izquierda el programa de tipo Launch que se desea ejecutar\n2. Hacer clic en el botón Ejecutar correspondiente\n3. Una vez terminado de cargar, seleccionar en el segundo desplegable uno de los scripts Python que deseamos ejecutar\n4. Hacer clic en el botón Ejecutar correspondiente\n5. Antes de ejecutar un nuevo programa de tipo 'Launch', selecionar la ventana terminal donde se está ejecutando este programa y presionar a la vez las teclas 'Ctrl' y 'c' para cerrar el programa launch'\n6. En la mayoría de los casos no es necesario cerrar manualmente los scripts de Python pero, si lo fuera, el procedimiento es siempre presionar a la vez las teclas 'Ctrl' y 'c'\n7. Para cerrar este programa, antes es necesario cerrar el programa de tipo Launch y luego repetir nuevamente el presionar a la vez las teclas 'Ctrl' y 'c'. La ventana de la GUI se cerrará en cuanto mováis el cursor encima de la misma."

        self.btn = Button(self.frame1, text="Ejecutar Python", command=self.start_run)
        self.btn.grid(row=1, column=2)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'
        
        self.btn = Button(self.frame1, text="Ejecutar Launch", command=self.start_run2)
        self.btn.grid(row=1, column=1)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'
        
        #self.close = Button(self.win, text="Cerrar Py", command=self.close_p)
        #self.close.grid(row=2, column=2)

        #self.close = Button(self.win, text="Cerrar Launch", command=self.close_p2)
        #self.close.grid(row=2, column=1)

        self.t2 = None
        self.log_data = Text(self.win, width=100, height=14) #tamaño de la celda grande tips
        self.log_data.grid(row=1, column=0)
        self.log_data.insert(END, self.tips)
        # self.code_list = ttk.Combobox(self.win, width=15)
        # self.code_list["value"] = ("reconocimiento de color", "reconocimiento de objetos", "Identificación de código QR")
        # self.code_list.current(0)
        # self.code_list.grid(row=1, column=1)
        self.log_data = Frame(self.win, width=267, height=167)
        #self.log_data.pack()
        self.log_data.place(anchor='nw', relx=0.80, rely=0.890)
        # Crear un objeto de tkinter ImageTk
        self.img = ImageTk.PhotoImage(Image.open("/home/catkin_ws/src/leial-test/lite6_moveit_config/pictures/logo_azul_reducido.png"))
        # Cree un widget de etiqueta para mostrar el texto o la imagen
        self.label = Label(self.log_data, image = self.img)
        self.label.grid(row=26, column=3)
        #self.label.pack()

# introduccion de una celda donde poner el desplazamiento del TCP

        self.frame2 = Frame(self.win, width=595, height=300)
        self.frame2.grid(row=2, column=0, padx=5, pady=5)

        self.label1 = Label(self.frame2, text="X Y Z", font=self.myFont, compound=CENTER)
        self.label1.grid(row=1, column=0, columnspan=3, padx=5, pady=5)
        #self.label1['background']='#006FB9'
        #self.label1['foreground']='#FFFFFF'

        self.label1 = Label(self.frame2, text="R P Y", font=self.myFont, compound=CENTER)
        self.label1.grid(row=1, column=4, columnspan=3, padx=5, pady=5)
        #self.label1['background']='#006FB9'
        #self.label1['foreground']='#FFFFFF'

        self.flecha_arriba_x = Image.open('/home/catkin_ws/src/leial-test/lite6_moveit_config/pictures/flecha_arriba.png')
        self.flecha_arriba_x = self.flecha_arriba_x.resize((45,70), Image.ANTIALIAS)
        self.img_x_pos= ImageTk.PhotoImage(self.flecha_arriba_x)
        self.flecha_arriba_x = Button(self.frame2, image=self.img_x_pos, text="+10mm X", compound= TOP, command=self.mueve_10mm_x)
        self.flecha_arriba_x.grid(row=0, column=1)
        #self.flecha_arriba_x['background']='#006FB9'
        #self.flecha_arriba_x['foreground']='#000000'
        #self.flecha_arriba_x['border']='5'
        
        self.flecha_izquierda_y = Image.open('/home/catkin_ws/src/leial-test/lite6_moveit_config/pictures/flecha_izquierda.png')
        self.flecha_izquierda_y = self.flecha_izquierda_y.resize((90,45), Image.ANTIALIAS)
        self.img_y_pos= ImageTk.PhotoImage(self.flecha_izquierda_y)
        self.flecha_izquierda_y = Button(self.frame2, image=self.img_y_pos, text="+10mm Y", compound= BOTTOM, command=self.mueve_10mm_y)
        self.flecha_izquierda_y.grid(row=2, column=0)
        #self.flecha_izquierda_y['background']='#006FB9'
        #self.flecha_izquierda_y['foreground']='#000000'
        #self.flecha_izquierda_y['border']='5'

        self.flecha_arriba_z = Image.open('/home/catkin_ws/src/leial-test/lite6_moveit_config/pictures/flecha_arriba.png')
        self.flecha_arriba_z = self.flecha_arriba_z.resize((45,70), Image.ANTIALIAS)
        self.img_z_pos= ImageTk.PhotoImage(self.flecha_arriba_z)
        self.flecha_arriba_z = Button(self.frame2, image=self.img_z_pos, text="+10mm Z", compound= TOP, command=self.mueve_10mm_z)
        self.flecha_arriba_z.grid(row=0, column=2)
        #self.flecha_arriba_z['background']='#006FB9'
        #self.flecha_arriba_z['foreground']='#000000'
        #self.flecha_arriba_z['border']='5'

        self.flecha_abajo_x = Image.open('/home/catkin_ws/src/leial-test/lite6_moveit_config/pictures/flecha_abajo.png')
        self.flecha_abajo_x = self.flecha_abajo_x.resize((45,70), Image.ANTIALIAS)
        self.img_x_neg= ImageTk.PhotoImage(self.flecha_abajo_x)
        self.flecha_abajo_x = Button(self.frame2, image=self.img_x_neg, text="-10mm X", compound= BOTTOM, command=self.mueve_10mm_neg_x)
        self.flecha_abajo_x.grid(row=2, column=1)
        #self.flecha_abajo_x['background']='#006FB9'
        #self.flecha_abajo_x['foreground']='#000000'
        #self.flecha_abajo_x['border']='5'

        self.flecha_derecha_y = Image.open('/home/catkin_ws/src/leial-test/lite6_moveit_config/pictures/flecha_derecha.png')
        self.flecha_derecha_y = self.flecha_derecha_y.resize((90,45), Image.ANTIALIAS)
        self.img_y_neg= ImageTk.PhotoImage(self.flecha_derecha_y)
        self.flecha_derecha_y = Button(self.frame2, image=self.img_y_neg, text="-10mm Y", compound= BOTTOM, command=self.mueve_10mm_neg_y)
        self.flecha_derecha_y.grid(row=2, column=2)
        #self.flecha_derecha_y['background']='#006FB9'
        #self.flecha_derecha_y['foreground']='#000000'
        #self.flecha_derecha_y['border']='5'

        self.flecha_abajo_z = Image.open('/home/catkin_ws/src/leial-test/lite6_moveit_config/pictures/flecha_abajo.png')
        self.flecha_abajo_z = self.flecha_abajo_z.resize((45,70), Image.ANTIALIAS)
        self.img_z_neg= ImageTk.PhotoImage(self.flecha_abajo_z)
        self.flecha_abajo_z = Button(self.frame2, image=self.img_z_neg, text="-10mm Z", compound= TOP, command=self.mueve_10mm_neg_z)
        self.flecha_abajo_z.grid(row=0, column=0)
        #self.flecha_abajo_z['background']='#006FB9'
        #self.flecha_abajo_z['foreground']='#000000'
        #self.flecha_abajo_z['border']='5'
        
        self.label1 = Label(self.frame2, text=" ", font=self.myFont, compound=CENTER)
        self.label1.grid(row=1, column=3, padx=3, pady=3)
        #self.label1['background']='#006FB9'
        #self.label1['foreground']='#FFFFFF'
        

        self.flecha_arriba_beta = Image.open('/home/catkin_ws/src/leial-test/lite6_moveit_config/pictures/flecha_arriba.png')
        self.flecha_arriba_beta = self.flecha_arriba_beta.resize((45,70), Image.ANTIALIAS)
        self.img_beta_pos= ImageTk.PhotoImage(self.flecha_arriba_beta)
        self.flecha_arriba_beta = Button(self.frame2, image=self.img_beta_pos, text="+15 grados Y", compound= TOP, command=self.orienta_15g_beta)
        self.flecha_arriba_beta.grid(row=0, column=5)
        #self.flecha_arriba_beta['background']='#006FB9'
        #self.flecha_arriba_beta['foreground']='#000000'
        #self.flecha_arriba_beta['border']='5'

        self.flecha_izquierda_alpha = Image.open('/home/catkin_ws/src/leial-test/lite6_moveit_config/pictures/flecha_izquierda.png')
        self.flecha_izquierda_alpha = self.flecha_izquierda_alpha.resize((90,45), Image.ANTIALIAS)
        self.img_alpha_pos= ImageTk.PhotoImage(self.flecha_izquierda_alpha)
        self.flecha_izquierda_alpha = Button(self.frame2, image=self.img_alpha_pos, text="+15 grados X", compound= BOTTOM, command=self.orienta_15g_alpha)
        self.flecha_izquierda_alpha.grid(row=2, column=4)
        #self.flecha_izquierda_alpha['background']='#006FB9'
        #self.flecha_izquierda_alpha['foreground']='#000000'
        #self.flecha_izquierda_alpha['border']='5'

        self.flecha_arriba_gamma = Image.open('/home/catkin_ws/src/leial-test/lite6_moveit_config/pictures/flecha_arriba.png')
        self.flecha_arriba_gamma = self.flecha_arriba_gamma.resize((45,70), Image.ANTIALIAS)
        self.img_gamma_pos= ImageTk.PhotoImage(self.flecha_arriba_gamma)
        self.flecha_arriba_gamma = Button(self.frame2, image=self.img_gamma_pos, text="+15 grados Z", compound= TOP, command=self.orienta_15g_gamma)
        self.flecha_arriba_gamma.grid(row=0, column=6)
        #self.flecha_arriba_gamma['background']='#006FB9'
        #self.flecha_arriba_gamma['foreground']='#000000'
        #self.flecha_arriba_gamma['border']='5'

        self.flecha_abajo_beta = Image.open('/home/catkin_ws/src/leial-test/lite6_moveit_config/pictures/flecha_abajo.png')
        self.flecha_abajo_beta = self.flecha_abajo_beta.resize((45,70), Image.ANTIALIAS)
        self.img_beta_neg= ImageTk.PhotoImage(self.flecha_abajo_beta)
        self.flecha_abajo_beta = Button(self.frame2, image=self.img_beta_neg, text="-15 grados Y", compound= BOTTOM, command=self.orienta_15g_neg_beta)
        self.flecha_abajo_beta.grid(row=2, column=5)
        #self.flecha_abajo_beta['background']='#006FB9'
        #self.flecha_abajo_beta['foreground']='#000000'
        #self.flecha_abajo_beta['border']='5'
        
        self.flecha_derecha_alpha = Image.open('/home/catkin_ws/src/leial-test/lite6_moveit_config/pictures/flecha_derecha.png')
        self.flecha_derecha_alpha = self.flecha_derecha_alpha.resize((90,45), Image.ANTIALIAS)
        self.img_alpha_neg= ImageTk.PhotoImage(self.flecha_derecha_alpha)
        self.flecha_derecha_alpha = Button(self.frame2, image=self.img_alpha_neg, text="-15 grados X", compound= BOTTOM, command=self.orienta_15g_neg_alpha)
        self.flecha_derecha_alpha.grid(row=2, column=6)
        #self.flecha_derecha_alpha['background']='#006FB9'
        #self.flecha_derecha_alpha['foreground']='#000000'
        #self.flecha_derecha_alpha['border']='5'

        self.flecha_abajo_gamma = Image.open('/home/catkin_ws/src/leial-test/lite6_moveit_config/pictures/flecha_abajo.png')
        self.flecha_abajo_gamma = self.flecha_abajo_gamma.resize((45,70), Image.ANTIALIAS)
        self.img_gamma_neg= ImageTk.PhotoImage(self.flecha_abajo_gamma)
        self.flecha_abajo_gamma = Button(self.frame2, image=self.img_gamma_neg, text="-15 grados Z", compound= TOP, command=self.orienta_15g_neg_gamma)
        self.flecha_abajo_gamma.grid(row=0, column=4)
        #self.flecha_abajo_gamma['background']='#006FB9'
        #self.flecha_abajo_gamma['foreground']='#000000'
        #self.flecha_abajo_gamma['border']='5'

        self.frame3 = Frame(self.win, width=250, height=300)
        self.frame3.grid(row=3, column=0, padx=5, pady=5)

        self.label1 = Label(self.frame3, text="Desplazam. deseado", font=self.myFont, compound=BOTTOM)
        self.label1.grid(row=0, column=0, padx=5, pady=5)
        #self.label1['background']='#006FB9'
        #self.label1['foreground']='#FFFFFF'
        self.entry1 = Entry(self.frame3)
        self.entry1.grid(row=1, column=0)
        
        global desplazamiento
        desplazamiento = self.entry1

        #self.btn = Button(self.win, text="mycobot320", command=self.grupo_mycobot320)
        #self.btn.grid(row=22, column=3)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'

        #self.btn = Button(self.win, text="lite 6", command=self.grupo_lite6)
        #self.btn.grid(row=23, column=3)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'
        
        self.btn = Button(self.frame3, text="Mueve X(mm)", command=self.mueve_x)
        self.btn.grid(row=2, column=0)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'

        self.btn = Button(self.frame3, text="Mueve Y(mm)", command=self.mueve_y)
        self.btn.grid(row=3, column=0)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'

        self.btn = Button(self.frame3, text="Mueve Z(mm)", command=self.mueve_z)
        self.btn.grid(row=4, column=0)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'

        
        self.label3 = Label(self.frame3, text="Coorden. deseadas", font=self.myFont, compound=BOTTOM)
        self.label3.grid(row=0, column=1, columnspan=2, padx=5, pady=5)
        #self.label3['background']='#006FB9'
        #self.label3['foreground']='#FFFFFF'
        self.entry3 = Entry(self.frame3, width=12)
        self.entry3.grid(row=1, column=1)
        global coord_x
        coord_x = self.entry3
        self.entry4 = Entry(self.frame3, width=12)
        self.entry4.grid(row=2, column=1)
        global coord_y
        coord_y = self.entry4
        self.entry5 = Entry(self.frame3, width=12)
        self.entry5.grid(row=3, column=1)
        global coord_z
        coord_z = self.entry5

        self.btn = Button(self.frame3, text="Mueve coord", command=self.mueve_coord)
        self.btn.grid(row=4, column=1, columnspan=2)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'


        self.espacio = Label(self.frame3, text="X(mm)", font=self.myFont, compound=LEFT)
        self.espacio.grid(row=1, column=2)
        #self.espacio['background']='#006FB9'
        #self.espacio['foreground']='#FFFFFF'
        #self.espacio['border']='0'
        
        self.espacio = Label(self.frame3, text="Y(mm)", font=self.myFont, compound=LEFT)
        self.espacio.grid(row=2, column=2)
        #self.espacio['background']='#006FB9'
        #self.espacio['foreground']='#FFFFFF'
        #self.espacio['border']='0'

        self.espacio = Label(self.frame3, text="Z(mm)", font=self.myFont, compound=LEFT)
        self.espacio.grid(row=3, column=2)
        #self.espacio['background']='#006FB9'
        #self.espacio['foreground']='#FFFFFF'
        #self.espacio['border']='0'

        self.frame4 = Frame(self.win, width=250, height=300)
        self.frame4.grid(row=4, column=0, padx=5, pady=5)

        self.btn = Button(self.frame4, text="Copia posición", command=self.copia_posicion)
        self.btn.grid(row=0, column=0)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'

        self.btn = Button(self.frame4, text="Copia angulos", command=self.copia_angulos)
        self.btn.grid(row=0, column=1)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'

        self.frame5 = Frame(self.win, width=250, height=300)
        self.frame5.grid(row=5, column=0, padx=5, pady=5)

        self.log_data2 = Text(self.frame5, width=50, height=7) #tamaño de la celda
        self.log_data2.grid(row=0, column=0)
        global posicion
        posicion = self.log_data2

        self.label2 = Label(self.frame3, text="Rotación deseada", font=self.myFont, compound=BOTTOM)
        self.label2.grid(row=0, column=3, padx=5, pady=5)
        #self.label2['background']='#006FB9'
        #self.label2['foreground']='#FFFFFF'
        self.entry2 = Entry(self.frame3)
        self.entry2.grid(row=1, column=3)
        
        global rotacion
        rotacion = self.entry2

        self.btn = Button(self.frame3, text="Orienta Y (grados)", command=self.orienta_beta)
        self.btn.grid(row=3, column=3)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'

        self.btn = Button(self.frame3, text="Orienta X (grados)", command=self.orienta_alpha)
        self.btn.grid(row=2, column=3)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'

        self.btn = Button(self.frame3, text="Orienta Z (grados)", command=self.orienta_gamma)
        self.btn.grid(row=4, column=3)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'

        #self.btn = Button(self.win, text="+100mm X", command=self.mueve_100mm_x)
        #self.btn.grid(row=24, column=0)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'

        #self.btn = Button(self.win, text="+100mm Y", command=self.mueve_100mm_y)
        #self.btn.grid(row=24, column=1)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'

        #self.btn = Button(self.win, text="+100mm Z", command=self.mueve_100mm_z)
        #self.btn.grid(row=24, column=2)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'

        #self.btn = Button(self.win, text="-100mm X", command=self.mueve_100mm_neg_x)
        #self.btn.grid(row=25, column=0)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'

        #self.btn = Button(self.win, text="-100mm Y", command=self.mueve_100mm_neg_y)
        #self.btn.grid(row=25, column=1)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'

        #self.btn = Button(self.win, text="-100mm Z", command=self.mueve_100mm_neg_z)
        #self.btn.grid(row=25, column=2)
        #self.btn['background']='#2B2A29'
        #self.btn['foreground']='#FFFFFF'
        
        
        


        

    def start_run(self):
        try:
            print(u"iniciar operación Python")
            one = self.myCombox.get()
            if one == u"reconocimiento de color pinza":
                self.run_py = "detect_obj_color_pinza.py"
                t1 = threading.Thread(target=self.open_py1)
                t1.setDaemon(True)
                t1.start()
            #elif one == u"reconocimiento de objetos":
            #    self.run_py = "detect_obj_img_pinza.py"
            #    t1 = threading.Thread(target=self.open_py)
            #    t1.setDaemon(True)
            #    t1.start()
            #elif one == u"Identificación de código QR":
            #    self.run_py = "detect_encode.py"
            #    t1 = threading.Thread(target=self.open_py2)
            #    t1.setDaemon(True)
            #    t1.start()
            #elif one == u"Ventosa que coge cubo":
            #    self.run_py = "mycobot320_legrip_pi_pl.py"
            #    t1 = threading.Thread(target=self.open_pysuction)
            #    t1.setDaemon(True)
            #    t1.start()
            #elif one == u"Agregar imagen de objeto":
            #    self.run_py = "add_img.py"
            #    t1 = threading.Thread(target=self.add_img)
            #    t1.setDaemon(True)
            #    t1.start()
            #elif one == u"Script Python con Moveit Commander":
            #    self.run_py = "mycobot320_legrip_pi_pl2.py"
            #    t1 = threading.Thread(target=self.python_moveitcommander)
            #    t1.setDaemon(True)
            #    t1.start()
            elif one == u"Evitar Obstaculos":
                self.run_py = "mycobot320_avoid.py"
                t1 = threading.Thread(target=self.python_avoid)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Moveit Commander Command Line":
                self.run_py = "moveit_commander_cmdline.py"
                t1 = threading.Thread(target=self.python_moveitcommander_cmdline)
                t1.setDaemon(True)
                t1.start()
            #elif one == u"Robot Escribe LEIAL":
            #    self.run_py = "mycobot320_writing.py"
            #    t1 = threading.Thread(target=self.python_writing)
            #    t1.setDaemon(True)
            #    t1.start()
            elif one == u"Lector Codigo de Barras 1":
                self.run_py = "barcodereader1.py"
                t1 = threading.Thread(target=self.python_barcodereader1)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Lector Codigo de Barras 2":
                self.run_py = "barcodereader2.py"
                t1 = threading.Thread(target=self.python_barcodereader2)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Brazo con Lector Codigo de Barras":
                self.run_py = "mycobot320_barcodereader.py"
                t1 = threading.Thread(target=self.python_brazo_barcodereader)
                t1.setDaemon(True)
                t1.start()
            #elif one == u"Robot pick and place":
            #    self.run_py = "mycobot320_pick_place.py"
            #    t1 = threading.Thread(target=self.python_picknplace)
            #    t1.setDaemon(True)
            #    t1.start()
            #elif one == u"reconocimiento de color pinza MoveIt":
            #    self.run_py = "detect_obj_color_moveit.py"
            #    t1 = threading.Thread(target=self.python_py1_moveit)
            #    t1.setDaemon(True)
            #    t1.start()
            elif one == u"Lite6 Robot Practicar soldaduras":
                self.run_py = "lite6_welding.py"
                t1 = threading.Thread(target=self.lite6_python_welding)
                t1.setDaemon(True)
                t1.start()


        except Exception as e:
            self.tips = str(e)
            self.log_data.insert(END, self.tips)
                    
    def start_run2(self):
        try:
            print(u"iniciar operación Launch")
            one = self.myCombox2.get()
            if one == u"Mycobot Deslizadores Pinza Fija":
                self.run_py = "mycobot_320_slider.launch"
                t1 = threading.Thread(target=self.rviz_desliza_fija)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Mycobot Deslizadores Pinza Movil":
                self.run_py = "mycobot_320_slider_gripper.launch"
                t1 = threading.Thread(target=self.rviz_desliza_movil)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Mycobot MoveIt Pinza Fija":
                self.run_py = "demo.launch"
                t1 = threading.Thread(target=self.rviz_moveit_fija)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Mycobot MoveIt Apagado Pinza Fija":
                self.run_py = "demo_moveit_apagado.launch"
                t1 = threading.Thread(target=self.rviz_moveit_apagado_fija)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Mycobot MoveIt Pinza Movil":
                self.run_py = "demo_pinza.launch"
                t1 = threading.Thread(target=self.rviz_moveit_movil)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Mycobot Verter Cerveza":
                self.run_py = "demo_pouring.launch"
                t1 = threading.Thread(target=self.simu_cerveza)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Mycobot Evitar Obstaculo":
                self.run_py = "demo_avoid.launch"
                t1 = threading.Thread(target=self.rviz_avoid)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Mycobot 2 Robots y cinta":
                self.run_py = "demo_robots.launch"
                t1 = threading.Thread(target=self.simu_2robocinta)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Lite6 Evitar Obstaculo":
                self.run_py = "demo_welding.launch"
                t1 = threading.Thread(target=self.rviz_lite6_avoid)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Mycobot Visión Artificial Moveit":
                self.run_py = "demo_vision_moveit_apagado.launch"
                t1 = threading.Thread(target=self.vis_art_moveit)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Lite6 Moveit pinza fija":
                self.run_py = "demo_welding.launch"
                t1 = threading.Thread(target=self.lite6_rviz_moveit_fija)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Lite6 Moveit pinza puntos":
                self.run_py = "demo_welding_spot.launch"
                t1 = threading.Thread(target=self.lite6_rviz_moveit_puntos)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Lite6 Moveit pinza laser":
                self.run_py = "demo_welding_laser.launch"
                t1 = threading.Thread(target=self.lite6_rviz_moveit_laser)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Lite6 Moveit pinza MIG/MAG":
                self.run_py = "demo_welding_mig.launch"
                t1 = threading.Thread(target=self.lite6_rviz_moveit_mig)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Lite6 Moveit pinza TIG":
                self.run_py = "demo_welding_tig.launch"
                t1 = threading.Thread(target=self.lite6_rviz_moveit_tig)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Lite6 Moveit pinza objeto":
                self.run_py = "demo_welding_objeto.launch"
                t1 = threading.Thread(target=self.lite6_rviz_moveit_objeto)
                t1.setDaemon(True)
                t1.start()
        except Exception as e:
            self.tips = str(e)
            self.log_data.insert(END, self.tips)

    def open_py(self):
        os.system(
            "python3 /home/catkin_ws/src/leial-test/mycobot_ai/scripts/detect_obj_img_pinza.py"
        )

    def open_py1(self):
        os.system(
            "python3 /home/catkin_ws/src/leial-test/mycobot_ai/scripts/detect_obj_color_pinza.py"
        )
    def python_py1_moveit(self):
        os.system(
            "python3 /home/catkin_ws/src/leial-test/mycobot_ai/scripts/detect_obj_color_pinza_moveit.py"
        )
    def open_py2(self):
        os.system(
            "python3 /home/catkin_ws/src/leial-test/mycobot_ai/scripts/detect_encode.py"
        )

    def add_img(self):
        os.system(
            "python3 /home/catkin_ws/src/leial-test/mycobot_ai/scripts/add_img.py"
        )
        
    def open_pysuction(self):
        os.system(
            "python3 /home/catkin_ws/src/leial-test/mycobot_320_moveit/scripts/mycobot320_legrip_pi_pl.py"
        )

    def python_moveitcommander(self):
        os.system(
            "python3 /home/catkin_ws/src/leial-test/mycobot_320_moveit/scripts/mycobot320_legrip_pi_pl2.py"
        )

    def python_avoid(self):
        os.system(
            "python3 /home/catkin_ws/src/leial-test/mycobot_320_moveit/scripts/mycobot320_avoid.py"
        )

    def python_moveitcommander_cmdline(self):
        os.system(
            "rosrun moveit_commander moveit_commander_cmdline.py"
        )

    def python_writing(self):
        os.system(
            "python3 /home/catkin_ws/src/leial-test/mycobot_320_moveit/scripts/mycobot320_writing.py"
        )
    def python_lite6_avoid(self):
        os.system(
            "python3 /home/catkin_ws/src/leial-test/lite6_moveit_config/scripts/lite6_avoid.py"
        )

    #def python_barcodereader1(self):
    #    os.system(
    #        "python ~/catkin_ws/src/mycobot_ros/lector_codigo_barras/barcodereader1.py"
    #    )

    #def python_barcodereader2(self):
    #    os.system(
    #        "python ~/catkin_ws/src/mycobot_ros/lector_codigo_barras/barcodereader2.py"
    #    )

    #def python_brazo_barcodereader(self):
    #    os.system(
    #        "python ~/catkin_ws/src/mycobot_ros/lector_codigo_barras/mycobot320_barcodereader.py"
    #    )

    
    def lite6_python_welding(self):
        os.system(
            "python3 /home/catkin_ws/src/leial-test/lite6_moveit_config/scripts/lite6_welding.py"
        )

    def roscore(self):
        os.system(
            "roscore"
        )
        
    def terminal(self):
        os.system(
            "gnome-terminal"
        )

    def vis_art(self):
        os.system(
            "roslaunch /home/catkin_ws/src/leial-test/mycobot_ai/launch/vision_320.launch"
        )

    def vis_art_moveit(self):
        os.system(
            "roslaunch /home/catkin_ws/src/leial-test/mycobot_320_moveit/launch/demo_vision_moveit_apagado.launch"
        )

    def rviz_desliza_fija(self):
        os.system(
            "roslaunch /home/catkin_ws/src/leial-test/mycobot_320/launch/mycobot_320_slider.launch"
        )
    
    def rviz_desliza_movil(self):
        os.system(
            "roslaunch /home/catkin_ws/src/leial-test/mycobot_320/launch/mycobot_320_slider_gripper.launch"
        )
    
    def rviz_moveit_fija(self):
        os.system(
            "roslaunch /home/catkin_ws/src/leial-test/mycobot_320_moveit/launch/demo.launch"
        )
    def rviz_moveit_apagado_fija(self):
        os.system(
            "roslaunch /home/catkin_ws/src/leial-test/mycobot_320_moveit/launch/demo_moveit_apagado.launch"
        )
    def rviz_moveit_movil(self):
        os.system(
            "roslaunch /home/catkin_ws/src/leial-test/mycobot_320_moveit/launch/demo_pinza.launch"
        )
    def lite6_rviz_moveit_fija(self):
        os.system(
            "roslaunch /home/catkin_ws/src/leial-test/lite6_moveit_config/launch/demo_welding.launch"
        )
    def lite6_rviz_moveit_puntos(self):
        os.system(
            "roslaunch /home/catkin_ws/src/leial-test/lite6_moveit_config/launch/demo_welding_spot.launch"
        )
    def lite6_rviz_moveit_laser(self):
        os.system(
            "roslaunch /home/catkin_ws/src/leial-test/lite6_moveit_config/launch/demo_welding_laser.launch"
        )
    def lite6_rviz_moveit_mig(self):
        os.system(
            "roslaunch /home/catkin_ws/src/leial-test/lite6_moveit_config/launch/demo_welding_mig.launch"
        )
    def lite6_rviz_moveit_tig(self):
        os.system(
            "roslaunch /home/catkin_ws/src/leial-test/lite6_moveit_config/launch/demo_welding_tig.launch"
        )
    def lite6_rviz_moveit_objeto(self):
        os.system(
            "roslaunch /home/catkin_ws/src/leial-test/lite6_moveit_config/launch/demo_welding_objeto.launch"
        )
            
    def simu_cerveza(self):
        os.system(
            "roslaunch /home/catkin_ws/src/leial-test/mycobot_320_moveit/launch/demo_pouring.launch"
        )
        
    def rviz_avoid(self):
        os.system(
            "roslaunch /home/catkin_ws/src/leial-test/mycobot_320_moveit/launch/demo.launch"
        )
    
    def simu_2robocinta(self):
        os.system(
            "roslaunch ~/catkin_ws/src/conveyor_demo/src/demo_world/launch/demo_robots.launch"
        )
    def rviz_lite6_avoid(self):
        os.system(
            "roslaunch /home/catkin_ws/src/leial-test/lite6_moveit_config/launch/demo_welding.launch"
        )
        
    def rviz_writing(self):
        os.system(
            "roslaunch ~/catkin_ws/src/myCobotROS/mycobot_320_moveit_pi_pl/launch/demo_writing.launch"
        )
    
    def grupo_mycobot320 (self):  
      #group = moveit_commander.MoveGroupCommander("arm_group")
    #self.group = group
      group_name = "arm_group"
      group = moveit_commander.MoveGroupCommander(group_name)
      self.group = group
      self.group_name = group_name
      return group, group_name

    def grupo_lite6 (self):  
      group_name = "lite6"
      group = moveit_commander.MoveGroupCommander(group_name)
      self.group = group
      self.group_name = group_name
      return group, group_name

    def mueve_x(self):
        try:
          
          tutorial = MoveGroupPythonIntefaceTutorial()
          cartesian_plan, fraction = tutorial.plan_cartesian_path_x()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def mueve_y(self):
        try:
          
          tutorial = MoveGroupPythonIntefaceTutorial()
          cartesian_plan, fraction = tutorial.plan_cartesian_path_y()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def mueve_z(self):
        try:
          
          tutorial = MoveGroupPythonIntefaceTutorial()
          cartesian_plan, fraction = tutorial.plan_cartesian_path_z()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def mueve_10mm_x(self):
        try:
          
          tutorial = MoveGroupPythonIntefaceTutorial()
          cartesian_plan, fraction = tutorial.plan_cartesian_path_x_10()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def mueve_10mm_y(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          cartesian_plan, fraction = tutorial.plan_cartesian_path_y_10()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def mueve_10mm_z(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          cartesian_plan, fraction = tutorial.plan_cartesian_path_z_10()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def mueve_10mm_neg_x(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          cartesian_plan, fraction = tutorial.plan_cartesian_path_neg_x()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def mueve_10mm_neg_y(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          cartesian_plan, fraction = tutorial.plan_cartesian_path_neg_y()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def mueve_10mm_neg_z(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          cartesian_plan, fraction = tutorial.plan_cartesian_path_neg_z()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def mueve_coord(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          tutorial.mueve_coord2()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def copia_posicion(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          tutorial.copia_posicion2()
          print ("Posición copiada!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def copia_angulos(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          tutorial.copia_angulos2()
          print ("Posición copiada!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def orienta_15g_beta(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          tutorial.mueve_15g_beta()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def orienta_15g_alpha(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          tutorial.mueve_15g_alpha()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def orienta_15g_gamma(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          tutorial.mueve_15g_gamma()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def orienta_15g_neg_beta(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          tutorial.mueve_15g_neg_beta()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def orienta_15g_neg_alpha(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          tutorial.mueve_15g_neg_alpha()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def orienta_15g_neg_gamma(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          tutorial.mueve_15g_neg_gamma()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def orienta_beta(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          tutorial.mueve_beta()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def orienta_alpha(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          tutorial.mueve_alpha()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def orienta_gamma(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          tutorial.mueve_gamma()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return
        
    def mueve_100mm_x(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          cartesian_plan, fraction = tutorial.plan_cartesian_path_x_100()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def mueve_100mm_y(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          cartesian_plan, fraction = tutorial.plan_cartesian_path_y_100()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def mueve_100mm_z(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          cartesian_plan, fraction = tutorial.plan_cartesian_path_z_100()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def mueve_100mm_neg_x(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          cartesian_plan, fraction = tutorial.plan_cartesian_path_neg_x_100()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def mueve_100mm_neg_y(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          cartesian_plan, fraction = tutorial.plan_cartesian_path_neg_y_100()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def mueve_100mm_neg_z(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          cartesian_plan, fraction = tutorial.plan_cartesian_path_neg_z_100()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return

    def mueve_alrededor_z_pos(self):
        try:
          tutorial = MoveGroupPythonIntefaceTutorial()
          tutorial.alrededor_z_pos()
          print ("Movimiento completado!")
        except rospy.ROSInterruptException:
          return
        except KeyboardInterrupt:
          return
                   
    def get_current_time(self):
        # tiempo de registro
        """Obtener la hora actual con formato."""
        current_time = time.strftime("%Y-%m-%d %H:%M:%S",
                                     time.localtime(time.time()))
        return current_time

    def write_log_to_Text(self, logmsg):
        # establecer la función de registro
        global LOG_NUM
        current_time = self.get_current_time()
        logmsg_in = str(current_time) + " " + str(logmsg) + "\n"  # nueva línea

        if LOG_NUM <= 18:
            self.log_data_Text.insert(END, logmsg_in)
            LOG_NUM += len(logmsg_in.split("\n"))
            # print(LOG_NUM)
        else:
            self.log_data_Text.insert(END, logmsg_in)
            self.log_data_Text.yview("fin")

    def run(self):
        self.win.mainloop()
        

    def tipo_grupo(self):
        print("Elige el tipo de grupo que deseas utilizar")
        print("0=mycobot320,           1=lite 6")
        seleccion = raw_input("")
        if seleccion == '0':
            grupo = self.grupo_mycobot320()
        if seleccion == '1':
            grupo = self.grupo_lite6()
        return grupo
    
    
    
    
    
    


if __name__ == "__main__":
    mc = Application()
    mc.run()
