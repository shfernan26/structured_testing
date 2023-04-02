#!/usr/bin/env python2
from __future__ import print_function, division
import Tkinter as tk
import rclpy
from rclpy.node import Node
from common.msg import radar_object_data
from time import sleep
from math import sin, cos, pi
import random

def dlength_processing(dlength):
	color = 'black'
	if dlength < 2.00: 
		color = 'red'
	elif dlength < 4.00: 
		color = 'blue'
	elif dlength < 6.00: 
		color = 'yellow'
	else: 
		color = 'green'
	return color

def determ_height(dz):
	if dz < -2:
		return 'red'
	elif dz < 1:
		return 'orange'
	elif dz < 4:
		return 'yellow'
	elif dz < 7:
		return 'blue'
	else:
		return 'purple'

def label_wexist(w):
	return 'white' if w > 0.5 else 'brown'

class Dashboard(object):

	def __init__(self, sub_topic_name, message_type):
		self.root = tk.Tk()
		self.canvas = tk.Canvas(self.root, width=800, height=600, background='#000000')
		self.canvas.create_line(0, 550, 800, 550, fill='#ffffff', tags='marker')
		self.canvas.create_line(400, 0, 400, 600, fill='#ffffff', tags='marker')
		# change the following for how long each "tick" would represent
		# I made scaling dynamic so everything else should change with them
		self.longitudinal_step = 25
		self.lateral_step = 1
		# horizontal markers
		for i in range(1, 5):
			x = 400-90*i
			y_start, y_end = 550, 530
			self.canvas.create_line(x, y_start, x, y_end, fill='#ffffff', tags='marker')
			self.canvas.create_text(x, y_start+10, fill='#ffffff', text=str(self.lateral_step*i), tags='marker')
			x = 400+90*i
			self.canvas.create_line(x, y_start, x, y_end, fill='#ffffff', tags='marker')
			self.canvas.create_text(x, y_start+10, fill='#ffffff', text=str(-self.lateral_step*i), tags='marker')
		# vertical markers
		for i in range(1, 9):
			x_start, x_end = 400, 420
			y = 550-65*i
			self.canvas.create_line(x_start, y, x_end, y, fill='#ffffff', tags='marker')
			self.canvas.create_text(x_start-15, y, fill='#ffffff', text=str(self.longitudinal_step*i), tags='marker')

		self.canvas.pack()

		node = Node('dashboard')
		rclpy.init()   
		rclpy.create_subscription(message_type,sub_topic_name, callback=self.add_to_buffer)

		self.front_radar_buffer = []	# list of radar messages
		self.front_radar_count = 0
		print('initialized')
	
	def update_canvas(self, sensor_tag, flag_test):
		self.canvas.delete(sensor_tag)
		if sensor_tag == 'front_radar':
			buffer_choice = self.front_radar_buffer
			self.front_radar_count = 0
		for i in buffer_choice:
			if flag_test == 'dLength':
				color = dlength_processing(i.dLength)
			elif flag_test == 'wExist':
				color = label_wexist(i.wExist)
			# TODO:
			# elif ... to add more signals
			self.canvas.create_rectangle(i.RadarDy-5, i.RadarDx-7,
										 i.RadarDy+5, i.RadarDx+7,
										 fill=color, tags=sensor_tag)
		buffer_choice *= 0	# cleans the referenced list
		self.canvas.update()
	
	def add_to_buffer(self, sensor_message):
		if sensor_message.radar_num == 2: # correct the number if wrong
			sensor_message.RadarDx = 550 - int(sensor_message.RadarDx/self.longitudinal_step*65)
			sensor_message.RadarDy = 400 + int(-sensor_message.RadarDy/self.lateral_step*90)
			self.front_radar_buffer.append(sensor_message)
			self.front_radar_count += 1

	def check_buffer(self):
		# change this for a different signal
		testing_flag = 'wExist'
		# change to the number of tracks per cycle
		if self.front_radar_count > 31:
			self.update_canvas('front_radar', testing_flag)
		# TODO:
		# if ... to add more sensors
	
if __name__ == '__main__':
	board = Dashboard('can_rx', radar_object_data)
	# if the dashboard is "lagging behind" increase the refresh rate
	rate = rclpy.Rate(30)
	while rclpy.ok():
		board.check_buffer()
		rate.sleep()
