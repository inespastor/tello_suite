#!/usr/bin/env python3
# This file is part of rosgpt package.; Based on the work of 2023 Anis Koubaa.;  rosgpt_client_node.py

import json
import ollama
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
import requests
from plugin_server_base.plugin_base import PluginBase


class ROSGPTClient(PluginBase):
    def __init__(self):
        super().__init__('rosgpt_client')
        self.declare_parameter('server_url', 'http://localhost:5000/rosgpt')
        self.server_url = self.get_parameter('server_url').value

        self.get_logger().info('ROSGPT client node started')

        self.send_text_command()

    def send_text_command(self):
        """
        Sends a text command to the ROSGPT system and receives a response from the ChatGPT language model.
        """
        while rclpy.ok():
            print('Enter a move command or a rotate command. The current ROSGPTParser of rosgpt_tello does not multiple command. Will be extended later')
            text_command = input("Enter a text command: ")
            data = {'text_command': text_command}

            response = requests.post(self.server_url, data=data)

            if response.status_code == 200:
                try:
                    response_str = response.content.decode('utf-8')
                    response_dict = json.loads(response_str)

                    self.get_logger().info('Response: {}'.format(response_dict['text']))
                    self.get_logger().info('JSON: {}'.format(json.loads(response_dict['json'])))
                except Exception as e:
                    print('[Exception] An unexpected error occurred:', str(e)) 
            else:
                self.get_logger().error('Error: {}'.format(response.status_code))


def main(args=None):
    rclpy.init(args=args)

    rosgpt_client = ROSGPTClient()

    rclpy.spin(rosgpt_client)

    rosgpt_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# source install/setup.bash
# ros2 launch tello_driver tello_driver.launch.py
# ros2 run nlp_control_plugin rosgpt
# ros2 run nlp_control_plugin rosgptparser_tello
# ros2 run nlp_control_plugin rosgpt_client_node 
