#!/usr/bin/env python3
# This file is part of rosgpt package.
# Based on the work of 2023 Anis Koubaa.

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty, Float32
from geometry_msgs.msg import Twist
from tello_msgs.msg import FlipControl
from plugin_server_base.plugin_base import PluginBase
import time


class TelloController(PluginBase):
    def __init__(self):
        super().__init__('tello_controller')
        self.create_subscription(String, '/text_cmd', self.text_cmd_callback, 10)
        self.battery_pub = self.create_publisher(Float32, '/battery_state', 10)
        self.cmd_pub = self.create_publisher(String, '/key_pressed', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/takeoff', 1)
        self.land_pub = self.create_publisher(Empty, '/land', 1)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.flip_pub = self.create_publisher(FlipControl, '/flip', 1)

        # Subscribe to tracking_info topic
        self.create_subscription(String, '/tracking_info', self.tracking_info_callback, 10)
        # Publisher for the filtered tracking info
        self.tracking_info_p = self.create_publisher(String, '/tracking_info_pilot_person', 10)

        # Initialize variables
        self.current_tracking_object = None  # Object to track, dynamically set by commands

        print('ROSGPT Tello Controller Started. Waiting for input commands ...')

    def text_cmd_callback(self, msg):
        try:
            # Parse incoming JSON command
            cmd = json.loads(msg.data)
            cmd = json.loads(cmd['json'])
            self.process_commands(cmd)
        except json.JSONDecodeError:
            print('[json.JSONDecodeError] Invalid or empty JSON string received:', msg.data)
        except Exception as e:
            print('[Exception] An unexpected error occurred:', str(e))

    def process_commands(self, cmds):
        for cmd in cmds:
            print("Processing command")
            self.process_command(cmd)
            time.sleep(8)

    def process_command(self, cmd):
        action = cmd['action']
        params = cmd['params']

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
            print('Drone is moving.')
        elif action == 'rotate':
            self.handle_rotate(params)
            print('Drone is rotating.')
        elif action == 'tracking':
            self.handle_tracking(params)
            print('Drone is starting to track.')
        elif action == 'stop_tracking':
            self.handle_stop_tracking()
            print('Stopped tracking.')
        elif action == 'switch_mode':
            self.handle_switch_mode(params)
            print(f"Switching to {params.get('mode')} control mode. Please select the image window.")
      

  
    def handle_stop_tracking(self):
        self.current_tracking_object = None
        stop_tracking_message = json.dumps({"action": "stop_tracking", "params": {}})
        # Publish the stop tracking
        self.tracking_info_p.publish(String(data=stop_tracking_message))
        print('Published stop tracking message to /tracking_info_pilot_person.')


    def handle_tracking(self, params):
        # Set the current object to track
        object_to_track = params.get('object', None)
        if object_to_track:
            self.current_tracking_object = object_to_track
            print(f'Starting to track object: {object_to_track}')
        else:
            print('No valid object specified for tracking.')


    def tracking_info_callback(self, msg):
        if not self.current_tracking_object:
            # No object set for tracking; do nothing
            print("No object specified for tracking.")
            return

        try:
            # Parse the incoming tracking_info message
            tracking_data = json.loads(msg.data)

            # Assume the object is not found initially
            object_found = False

            # Find the relevant object in the tracking data
            for item in tracking_data:
                if 'info' in item and 'objects' in item['info']:
                    if self.current_tracking_object in item['info']['objects']:
                        # Object is found, publish the filtered tracking information
                        filtered_info = json.dumps(item)
                        self.tracking_info_p.publish(String(data=filtered_info))
                        print(f'Published filtered tracking info: {filtered_info}')
                        object_found = True
                        break  # Stop after finding the first match

            if not object_found:
                print(f"Tracking object '{self.current_tracking_object}' not found in the current tracking data.")
        except json.JSONDecodeError:
            print('[JSONDecodeError] Failed to decode tracking_info message:', msg.data)
        except Exception as e:
            print('[Exception] An error occurred in tracking_info_callback:', str(e))


    def handle_switch_mode(self, params):
        mode = params.get('mode', 'keyboard')
        msg = String()
        msg.data = mode
        self.cmd_pub.publish(msg)  # Publish the new mode to the ControlStation
        print(f"Published mode switch command: {mode}")


    def handle_move(self, params):
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)
        z = params.get('z', 0.0)
        duration = params.get('time', 1.0)

        # Set velocity
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z

        # Publish velocity command
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

    def handle_rotate(self, params):
        z = params.get('z', 0.0)
        duration = params.get('time', 1.0)

        # Set angular velocity
        msg = Twist()
        msg.angular.z = z

        # Publish angular velocity command
        self.vel_pub.publish(msg)
        print(f'Drone rotating: z={z} rad/s, time={duration} s')

        # Sleep for the duration to maintain the rotation
        time.sleep(duration)

        # Stop the drone
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)
        print('Drone stopped rotating.')

    def handle_flip(self, params):
        direction = params.get('direction', 'forward')
        msg = FlipControl()

        # Map directions
        if direction == "left":
            msg.flip_left = True
        elif direction == "right":
            msg.flip_right = True
        elif direction == "forward":
            msg.flip_forward = True
        elif direction == "backward":
            msg.flip_backward = True

        # Publish the flip message
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
