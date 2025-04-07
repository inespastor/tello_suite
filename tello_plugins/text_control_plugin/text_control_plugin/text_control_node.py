#!/usr/bin/env python3

import rclpy
from text_control_plugin.text_control import TextControl #Instead of TextControlNode


def main(args=None):
    rclpy.init(args=args)

    node = TextControl() #Instead of TextControlNode

    rclpy.spin(node)
    rclpy.shutdown()
