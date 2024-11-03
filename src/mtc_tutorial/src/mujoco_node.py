import mujoco as mj
import mujoco.viewer
import time
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.msg import PlanningScene

max_width = 100
max_height = 100

record_mode = 0		# 1 - Record MoveIt trajectory , 0 - Play saved trajectory

class MujocoController(Node):
	def __init__(self, model, data, viewer, write_file_path, read_file_path, record_mode):
		super().__init__('mujoco_controller')
		
		self.subscription = self.create_subscription(
			PlanningScene,
			'monitored_planning_scene',	
			self.listener_callback,
			10
		)

		self.model = model
		self.data = data	
		self.viewer = viewer
		self.write_file_path = write_file_path
		self.read_file_path = read_file_path
		self.record_mode = record_mode

		self.data.ctrl[:] = [0,-np.pi/2,0,-np.pi/2,0,0,0]
		self.ctrl_angles = self.data.ctrl.copy()
		self.joint_angles = self.data.qpos[[-14,-13,-12,-11,-10,-9,-4]]
		self.joint_angles[6] = min((self.joint_angles[6]/0.7)*255, 255)	# Mapping to range of 0 - 255

		self.motions = []
		self.motion_count = 0
		self.traj_step = 0
		self.thresholds_coarse = np.array([0.3]*6 + [200])
		self.thresholds_fine = np.array([0.03]*6 + [200])
		if not self.record_mode:
			self.read_motions()
		
		self.timer_count = 0.000000001
		self.control_timer = self.create_timer(self.timer_count, self.control_callback)\

	def control_callback(self):
		start = time.time()
		if self.viewer.is_running():
			step_start = time.time()

			self.joint_angles = self.data.qpos[[-14,-13,-12,-11,-10,-9,-4]]
			self.joint_angles[6] = min((self.joint_angles[6]/0.7)*255, 255)	# Mapping to range of 0 - 255
			if not self.record_mode:
				self.execute_trajectory()
			mj.mj_step(self.model, self.data)

			self.viewer.sync()

			time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
			if time_until_next_step > 0:
				time.sleep(time_until_next_step)

	def listener_callback(self, msg):
		self.ctrl_angles = msg.robot_state.joint_state.position[:7]
		self.ctrl_angles[6] = min((self.ctrl_angles[6]/0.7)*255, 255)	# Mapping to range of 0 - 255

		if self.record_mode:
			self.data.ctrl[:] = list(self.ctrl_angles)
			with open(self.write_file_path, "a") as txt_file:
				ctrl_vals = [round(v,3) for v in self.data.ctrl]
				txt_file.writelines(f"{ctrl_vals[0]} {ctrl_vals[1]} {ctrl_vals[2]} {ctrl_vals[3]} {ctrl_vals[4]} {ctrl_vals[5]} {ctrl_vals[6]}\n")	
	
	def read_motions(self):

		with open(self.read_file_path, "r") as file:
			trajectory = []
			for line in file:
				if line != "\n":
					target = [float(x) for x in line.split(" ")]
					trajectory.append(target)
				else:
					self.motions.append(trajectory)
					trajectory = []
			self.motions.append(trajectory)
			trajectory = []

	def execute_trajectory(self):
		if self.motion_count>=len(self.motions):
			return
		
		trajectory = self.motions[self.motion_count]
		target = np.array(trajectory[self.traj_step])
		current = np.array(self.joint_angles)
		diff = [abs(p) for p in current - target]
		thresh_check = diff < self.thresholds_coarse
		if self.traj_step == len(trajectory)-1:
			thresh_check = diff < self.thresholds_fine
			
		if not thresh_check.all():
			self.data.ctrl[:] = target
		else:
			self.traj_step+=1
			if self.traj_step >= len(trajectory):
				self.motion_count+=1
				self.traj_step = 0
		

def main():
	rclpy.init(args=None)

	trajectory_write_file = os.path.abspath(os.path.join(os.path.dirname( __file__ ), 'joint_values.txt'))
	trajectory_read_file = os.path.abspath(os.path.join(os.path.dirname( __file__ ), 'path.txt'))

	open(trajectory_write_file, 'w').close()
	model = mj.MjModel.from_xml_path(os.path.abspath(os.path.join(os.path.dirname( __file__ ), 'pick_place_scene.xml')))
	data = mj.MjData(model)

	with mj.viewer.launch_passive(model, data) as viewer:
		mujoco_controller = MujocoController(model, data, viewer, trajectory_write_file, trajectory_read_file, record_mode)
		rclpy.spin(mujoco_controller)
		mujoco_controller.destroy_node()
		rclpy.shutdown()


if __name__=="__main__":
	main()