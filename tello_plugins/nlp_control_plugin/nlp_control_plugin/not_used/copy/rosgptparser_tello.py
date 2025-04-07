#!/usr/bin/env python3
# This file is part of rosgpt package.; Based on the work of 2023 Anis Koubaa.

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty, Float32, Bool, Int32
from geometry_msgs.msg import Twist
from tello_msgs.msg import FlipControl
from plugin_server_base.plugin_base import PluginBase
import time

class TelloController(PluginBase):
    def __init__(self):
        super().__init__('tello_controller')
  
        self.create_subscription(String,'/text_cmd',self.text_cmd_callback,10)
                
        self.battery_pub = self.create_publisher(Float32, '/battery_state', 10)  
        self.cmd_pub = self.create_publisher(String, '/key_pressed', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/takeoff', 1)
        self.land_pub = self.create_publisher(Empty, '/land', 1)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) 
        self.flip_pub = self.create_publisher(FlipControl, '/flip', 1) 

        print('ROSGPT Tello Controller Started. Waiting for input commands ...')
     
    #def battery_callback(self, msg):
     #   self.battery_level_data = msg.percentage


    def text_cmd_callback(self, msg):
      
        #if self.battery_level_data is not None and self.battery_level_data < 15:
        #    print('Battery too low for operation.')
        #    return
        
        
        try:
            cmd = json.loads(msg.data)
            cmd = cmd['json']
            print(cmd)
            self.process_commands(cmd)

        except json.JSONDecodeError:
            print('[json.JSONDecodeError] Invalid or empty JSON string received:', msg.data)
        except Exception as e:
            print('[Exception] An unexpected error occurred:', str(e))


    def process_commands(self, cmds):
      
        for cmd in cmds:
            print("Sending command")
            self.process_command(cmd)
            time.sleep(8)


    def process_command(self, cmd):
        action = cmd.get('action', "")
        params = cmd.get('params', {})
        if action == 'takeoff':
            self.takeoff_pub.publish(Empty())
            print('Drone taking off.')
            
        elif action == 'land':
            self.land_pub.publish(Empty())
            print('Drone landing.')
        
        elif action == 'flip':
            self.handle_flip(params)
            print('Drone is flipping.')
            
        elif action == 'move':
            self.handle_move(params)

        elif action == 'rotate':
            self.handle_rotate(params)


#[{"action": "move", "params": {"x":"float", "y": float, "z": "float", "time": "float"}}]       


    def handle_move(self, params):
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)
        z = params.get('z', 0.0)
        duration = params.get('time', 1.0)

        # Set vel
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.linear.z = float(z)

        # Publish vel
        self.vel_pub.publish(msg)
        print(f'Drone moving: x={x} m/s, y={y} m/s, z={z} m/s, time={duration} s')

        # Sleep the time to maintain movement
        time.sleep(duration)

        # Stop the drone 
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        self.vel_pub.publish(msg)
        print('Drone stopped.')
        


#[{"action": "rotate", "params": {"z": "float", "time": "float"}}]      
    def handle_rotate(self, params):
        z = params.get('z', 0.0)
        duration = params.get('time', 1.0)

        # Set the angular velocity
        msg = Twist()
        msg.angular.z = float(z)

        # Publish the angular velocity command
        self.vel_pub.publish(msg)
        print(f'Drone rotating: z={z} rad/s, time={time} s')

        # Sleep for the duration to maintain the rotation
        time.sleep(duration)

        # Stop the drone by setting the angular velocity to zero
        msg.angular.z = 0.0

        self.vel_pub.publish(msg)
        print('Drone stopped rotating.')
    

    def handle_flip(self, params):
        direction = params.get('direction', 'forward')
        print(direction)
        msg = FlipControl()
        # Map directions         
        flip_params = {
        'forward': 'forward',
        'backward': 'back',
        'right': 'right',
        'left': 'left'}

        if direction == "left":
            msg.flip_left = True
        elif direction == "right":
            msg.flip_right = True
        elif direction == "forward":
            msg.flip_forward = True
        elif direction == "back":
            msg.flip_backward = True

        print(direction)
        # Create and publish the flip message
        
        
        self.flip_pub.publish(msg)
        print(f'Drone flipping: direction={direction}')

            
def main(args=None):
    rclpy.init(args=args)
    node = TelloController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()