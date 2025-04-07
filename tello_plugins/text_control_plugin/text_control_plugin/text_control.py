#!/usr/bin/env python3
import rclpy
import time
import math
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from plugin_server_base.plugin_base import PluginBase


class TextControl(PluginBase):
    def __init__(self):
        super().__init__('text_control')
        self.cmd_pub = self.create_publisher(String, '/key_pressed', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/takeoff', 1)
        self.land_pub = self.create_publisher(Empty, '/land', 1)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) 
        self.valid_commands = {'takeoff', 'land', 'forward', 'backward', 'right', 'left', 'up', 'down', 'rotate_r', 'rotate_l'}
    
    def tick(self):
        self.get_user_input()

    def get_user_input(self):
        user_input = input("Enter command for the drone valid commands with distance in meters or angles: 'takeoff', 'land', 'forward', 'backward', 'right', 'left', 'up', 'down', 'rotate_r', 'rotate_l'): ")
        parts = user_input.lower().strip().split()  
        command = parts[0]
        value = parts[1] if len(parts) > 1 else None
        

        if command not in self.valid_commands:
            print("Invalid command. Please enter a valid command.")
            return 
        
  
        if command == 'takeoff':
            self.take_off()
            print("Drone taking off")

        elif command == 'land':
            self.land()
            print("Drone landing")

        elif command in ['rotate_l', 'rotate_r']:
            angle = float(value) if value else 0.0
            self.handle_rotation_command(command, angle)

        else:
            distance = float(value) if value else 0.0
            self.handle_linear_command(command, distance)


    def handle_linear_command(self, command, value):
        msg = Twist()
        linear_velocity = 0.5
        duration = abs(value) / linear_velocity
        
        if command == 'forward':
            msg.linear.x = linear_velocity
        elif command == 'backward':
            msg.linear.x = -linear_velocity
        elif command == 'left':
            msg.linear.y = linear_velocity
        elif command == 'right':
            msg.linear.y = -linear_velocity
        elif command == 'up':
            msg.linear.z = linear_velocity
        elif command == 'down':
            msg.linear.z = -linear_velocity

        self.publish_twist_for_duration(msg, duration)

    def take_off(self):
        print('Drone takes off')

        self.takeoff_pub.publish(Empty())

    def land(self):
        print('Drone lands')
        self.land_pub.publish(Empty())


    def handle_rotation_command(self, command, angle):
        msg = Twist()
        angular_velocity = 0.5
        duration = abs(angle/70) / angular_velocity

        if command == 'rotate_l':
            msg.angular.z = angular_velocity
        elif command == 'rotate_r':
            msg.angular.z = -angular_velocity

        self.publish_twist_for_duration(msg, duration)

    def publish_twist_for_duration(self, twist_msg, duration):
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.vel_pub.publish(twist_msg)
            time.sleep(0.1)

    def publish_command(self, command, value):
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)

        if command not in ['rotate_l', 'rotate_r']:
            self.handle_linear_command(command, value)
        else:
            self.handle_rotation_command(command, value)

def main(args=None):
    rclpy.init(args=args)
    node = TextControl()
    node.get_user_input()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



#EXECUTE:
# ros2 launch tello_driver tello_driver.launch.py
# ros2 run text_control_plugin text_control_plugin --ros-args -p standalone:=true






#INTERESTING COMANDS
#ros2 node list
#ros2 node info /name_node
#ros2 topic list
#ros2 topic info /name_topic
#ros2 interface show /type_topic
#ros2 topic echo /name_topic
#ros2 interface show /type




'''
#SECOND TRY DISTANCE INCLUDED, NO ANGLES
#!/usr/bin/env python3

import rclpy
import time
import math
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from plugin_server_base.plugin_base import PluginBase


class TextControl(PluginBase):
    def __init__(self):
        super().__init__('text_control')
        self.cmd_pub = self.create_publisher(String, '/key_pressed', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/takeoff', 1)
        self.land_pub = self.create_publisher(Empty, '/land', 1)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) 
        self.valid_commands = {'takeoff', 'land', 'forward', 'backward', 'right', 'left', 'up', 'down', 'rotate left', 'rotate right'}
    
    
    def tick(self):
        self.get_user_input()

        
    def get_user_input(self):
        user_input = input("Enter command for the drone (valid commands: 'takeoff', 'land', 'forward', 'backward', 'right', 'left', 'up', 'down', 'rotate left', 'rotate right'): ")
        parts = user_input.lower().strip().split()  # Split the input into parts
        
        # Extract the command and distance from the input
        command = parts[0]
        distance = float(parts[1]) if len(parts) > 1 else 0.0  # Assume zero distance if not provided
        print("Command:", command)
        print("Distance:", distance)

        print(command)
        if command not in self.valid_commands:
            print("Invalid command. Please enter a valid command.")
            return 
        if command == 'takeoff':
            self.take_off()
            print("Drone taking off")
        elif command == 'land':
            self.land()
            print("Drone landing")
        else:
            self.publish_command(command, distance)

            
    def handle_twist_command(self, command, distance):
        # This method interprets string commands and publishes corresponding Twist messages
        distance=distance/10
        msg = Twist()
        linear_velocity = 0.5
        duration = distance / linear_velocity
        if command == 'forward':
            msg.linear.x = linear_velocity
            self.publish_twist_for_duration(msg, duration)
        elif command == 'backward':
            msg.linear.x = -linear_velocity
            self.publish_twist_for_duration(msg, duration)
        elif command == 'left':
            msg.linear.y = linear_velocity
            self.publish_twist_for_duration(msg, duration)
        elif command == 'right':
            msg.linear.y = -linear_velocity
            self.publish_twist_for_duration(msg, duration)
        elif command == 'up':
            msg.linear.z = linear_velocity
            self.publish_twist_for_duration(msg, duration)
        elif command == 'down':
            msg.linear.z = -linear_velocity
            self.publish_twist_for_duration(msg, duration)
        elif command == 'rotate left':
            msg.angular.z = 0.5
        elif command == 'rotate right':
            msg.angular.z = -0.5
        self.vel_pub.publish(msg)            


    def publish_twist_for_duration(self, twist_msg, duration):
    
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.vel_pub.publish(twist_msg)
            time.sleep(0.1)  


    def take_off(self):
        print('Drone takes off')

        self.takeoff_pub.publish(Empty())

    def land(self):
        print('Drone lands')
        self.land_pub.publish(Empty())

    def publish_command(self, command, distance):
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.handle_twist_command(command, distance)

def main(args=None):
    rclpy.init(args=args)
    node = TextControl()
    node.get_user_input()
    node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()

#############################3





#FIRST TRY NO DISTANCE/ANGLES
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from plugin_server_base.plugin_base import PluginBase


class TextControl(PluginBase):
    def __init__(self):
        super().__init__('text_control')
        self.is_airborne = False
        self.cmd_pub = self.create_publisher(String, '/key_pressed', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/takeoff', 1)
        self.land_pub = self.create_publisher(Empty, '/land', 1)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # Added this line
        self.valid_commands = {'takeoff', 'land', 'forward', 'backward', 'right', 'left', 'up', 'down', 'rotate left', 'rotate right'}
    
    
    def tick(self):
        self.get_user_input()

        
        
    def get_user_input(self):
        user_input = input("Enter command for the drone (valid commands: 'takeoff', 'land', 'up', 'down'): ")
        command = user_input.lower().strip()
        print(command)
        if command not in self.valid_commands:
            print("Invalid command. Please enter a valid command.")
            return 
        if command == 'takeoff':
            self.take_off()
         
        elif command == 'land':
            self.land()
              
        else:
            self.publish_command(command)
            
    def handle_twist_command(self, command):
        # This method interprets string commands and publishes corresponding Twist messages
        msg = Twist()
        if command == 'forward':
            msg.linear.x = 0.5
        elif command == 'backward':
            msg.linear.x = -0.5
        elif command == 'left':
            msg.linear.y = 0.5
        elif command == 'right':
            msg.linear.y = -0.5
        elif command == 'up':
            msg.linear.z = 0.5
        elif command == 'down':
            msg.linear.z = -0.5
        elif command == 'rotate left':
            msg.angular.z = 0.5
        elif command == 'rotate right':
            msg.angular.z = -0.5
        self.vel_pub.publish(msg)            


    def take_off(self):
        print('Drone takes off')

        self.takeoff_pub.publish(Empty())

    def land(self):
        print('Drone lands')
        self.land_pub.publish(Empty())

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.handle_twist_command(command)

def main(args=None):
    rclpy.init(args=args)
    node = TextControl()
    node.get_user_input()
    node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
'''
