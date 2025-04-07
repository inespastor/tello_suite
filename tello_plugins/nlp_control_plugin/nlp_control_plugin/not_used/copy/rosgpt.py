#!/usr/bin/env python3
# This file is part of rosgpt package.; Based on the work of 2023 Anis Koubaa.

import os
import json
import openai
import ollama
from ollama import Client
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, send_from_directory, jsonify
from flask_restful import Resource, Api
from flask_cors import CORS
import pyttsx3  # pip install pyttsx3 #you need to install libespeak1 on Ubuntu # sudo apt-get install libespeak1
from rclpy.executors import SingleThreadedExecutor
import subprocess
from plugin_server_base.plugin_base import PluginBase
from ament_index_python import get_package_share_directory

# Instantiate a Flask application object with the given name
app = Flask(__name__)

# Enable Cross-Origin Resource Sharing (CORS) for the Flask app
CORS(app)

# Create an API object that wraps the Flask app to handle RESTful requests
#api = Api(app)

#openai_api_key = os.getenv('OPENAI_API_KEY')
#spin_lock = threading.Lock()

#tts_engine = pyttsx3.init()

# Create a separate threading lock for synchronizing access to the TTS engine
tts_lock = threading.Lock()



client = Client(host='http://localhost:11434')


def speak(text):
    """
    This function uses the Text-to-Speech (TTS) engine to speak the given text.

    It is an optional method that can be used if you want the system to audibly
    communicate the text messages.

    Args:
        text (str): The text to be spoken by the TTS engine.

    Note:
        This method is optional and can be used when audible communication of text
        messages is desired. If not needed, it can be omitted from the implementation.
    """
    # Acquire the TTS lock to ensure exclusive access to the TTS engine
    with tts_lock:
        # Instruct the TTS engine to say the given text
        tts_engine.say(text)

        # Block and wait for the TTS engine to finish speaking
        tts_engine.runAndWait()



class ROSGPTNode(PluginBase):
    def __init__(self):
        """
        Initialize the ROSGPTNode class which is derived from the rclpy Node class.
        """
        # Call the superclass constructor and pass the name of the node
        super().__init__('chatgpt_ros2_node')
        # Create a publisher for the 'text_cmd' topic with a message queue size of 10
        self.publisher = self.create_publisher(String, 'text_cmd', 10)

    def publish_message(self, message):
        """
        Publish the given message to the 'text_cmd' topic.
        Args:
            message (str): The message to be published.
        """
        msg = String() # Create a new String message 
        msg.data = message # Convert the message to a JSON string and set the data field of the message
        self.publisher.publish(msg) # Publish the message using the publisher 
        #print('message Published: ', message) # Log the published message
        print('msg.data Published: ', msg.data) # Log the published message
        
        



def process_and_publish_chatgpt_response(chatgpt_ros2_node, text_command, chatgpt_response, use_executors=True):
    """
    Process the chatbot's response and publish it to the 'text_cmd' topic.

    Args:
        chatgpt_ros2_node (ROSGPTNode): The ROS2 node instance.
        text_command (str): The text command received from the user.
        chatgpt_response (str): The response from the chatbot.
        use_executors (bool, optional): Flag to indicate whether to use SingleThreadedExecutor. Defaults to True.
    """
    chatgpt_ros2_node.publish_message(chatgpt_response) # Publish the chatbot's response using the ROS2 node
    # If use_executors flag is True, use SingleThreadedExecutor
    if use_executors:
        executor = SingleThreadedExecutor()# Create a new executor for each request 
        executor.add_node(chatgpt_ros2_node) # Add the node to the executor
        executor.spin_once()#  Spin the executor once
        executor.remove_node(chatgpt_ros2_node) # Remove the node from the executor
    # If use_executors flag is False, use spin_lock to synchronize access
    else:
        with spin_lock:
            rclpy.spin_once(chatgpt_ros2_node)



class ROSGPTProxy(Resource):
    """
    A class derived from flask_restful.Resource, responsible for handling incoming HTTP POST requests.
    """

    def __init__(self, chatgpt_ros2_node):
        """
        Initialize the ROSGPTProxy class with the given ROS2 node.

        Args:
            chatgpt_ros2_node (ROSGPTNode): The ROS2 node instance.
        """
        self.chatgpt_ros2_node = chatgpt_ros2_node

    def askGPT(self, text_command):
        """
        Send a text command to the GPT-3 model and receive a response.
        Args:
            text_command (str): The text command to be sent to the GPT-3 model.
        Returns:
            str: The response from the GPT-3 model as a JSON string.
        """
        # Create the GPT-3 prompt with example inputs and desired outputs
        prompt =  """ Consider the following ontology:

                        [{"action": "takeoff", "params":{}}]
                        [{"action": "move", "params": {"x":"float", "y": float, "z": "float", "time": "float"}}]
                        [{"action": "rotate", "params": {"z": "float", "time": "float"}}]
                        [{"action": "flip", "params":{"direction":"str"}}]
                        [{"action": "land", "params":{}}]
                        [{"action": "recordings", "params":{"num":"str"}}]


                        You will be given human language prompts and should return JSON conforming to the ontology. Any action not in the ontology should be ignored. Here are some examples:

                        
                        Prompt: "Take off the drone."
                        Returns: [{"action": "takeoff", "params":{}}]

                        Prompt: "Departure the drone."
                        Returns: [{"action": "takeoff", "params":{}}]

                        Prompt: "Launch the drone."
                        Returns: [{"action": "takeoff", "params":{}}]

                        Prompt: "Land the drone."
                        Returns: [{"action": "land", "params":{}}]

                        Prompt: "Move the drone for 1 second."
                        Returns: [{"action": "move", "params": {"x":0.5, "y": 0, "z": 0, "time": 1}}]

                        Prompt: "Take off and land the drone."
                        Returns: [{"action": "takeoff", "params":{}}, {"action": "land", "params":{}}]

                        Prompt: "Take off the drone and move forward at 0.5 meters per second for 1 second."
                        Returns: [{"action": "takeoff", "params":{}}, {"action": "move", "params": {"x":0.5, "y": 0, "z": 0, "time": 1}}]

                        Prompt: "Take off the drone and go ahead at 0.5 meters per second for 1 second."
                        Returns: [{"action": "takeoff", "params":{}}, {"action": "move", "params": {"x":0.5, "y": 0, "z": 0, "time": 1}}]

                        Prompt: "Move the drone backward at 0.3 meters per second for 2 seconds."
                        Returns:[{"action": "move", "params": {"x": -0.3, "y": 0.0, "z": 0.0, "time": 2.0}}]
                        
                        Prompt: "Move the drone up at 0.8 meters per second for 0.5 seconds."
                        Returns: [{"action": "move", "params": {"x": 0.0, "y": 0.0, "z": 0.8, "time": 0.5}}]

                        Prompt: "Go up at 0.8 meters per second for 0.5 seconds."
                        Returns: [{"action": "move", "params": {"x": 0.0, "y": 0.0, "z": 0.8, "time": 0.5}}]
                        
                        Prompt: "Move the drone down at 0.7 meters per second for 0.3 seconds."
                        Returns: [{"action": "move", "params": {"x": 0.0, "y": 0.0, "z": -0.7, "time": 0.3}}]

                        Prompt: "Move the drone down at 0.7 meters per second for 0.7 seconds."
                        Returns: [{"action": "move", "params": {"x": 0.0, "y": 0.0, "z": -0.7, "time": 0.7}}]

                        Prompt: "Go down at 0.7 meters per second for 0.7 seconds."
                        Returns: [{"action": "move", "params": {"x": 0.0, "y": 0.0, "z": -0.7, "time": 0.7}}]

                        Prompt: "Move the drone to the left at 0.6 meters per second for 1 seconds."
                        Returns: [{"action": "move", "params": {"x": 0.0, "y": 0.6, "z": 0.0, "time": 1.0}}]

                        Prompt: "Go to the left at 0.6 meters per second for 1 seconds."
                        Returns: [{"action": "move", "params": {"x": 0.0, "y": 0.6, "z": 0.0, "time": 1.0}}]
                        
                        Prompt: "Move the drone to the right at 0.2 meters per second for 2 seconds."
                        Returns: [{"action": "move", "params": {"x": 0.0, "y": -0.2, "z": 0.0, "time": 2.0}}]

                        Prompt: "Go to the right at 0.2 meters per second for 2 seconds."
                        Returns: [{"action": "move", "params": {"x": 0.0, "y": -0.2, "z": 0.0, "time": 2.0}}]

                        Prompt: "Rotate the drone clockwise (to the right) at 1.0 degrees per second for 5 seconds."
                        Returns: [{"action": "rotate", "params": {"z": 1.0, "time": 5.0}}]

                        Prompt: "Turn the drone clockwise (to the right) at 1.0 degrees per second for 5 seconds."
                        Returns: [{"action": "rotate", "params": {"z": 1.0, "time": 5.0}}]
                        
                        Prompt: "Rotate the drone counterclockwise (to the left) at 3 degrees per second for 4 seconds."
                        Returns: [{"action": "rotate", "params": {"z": -3.0, "time": 4.0}}]

                        Prompt: "Rotate the drone to the left at 3 degrees per second for 4 seconds."
                        Returns: [{"action": "rotate", "params": {"z": -3.0, "time": 4.0}}]

                        Prompt: "Take off the drone and do one flip forward."
                        Returns: [{"action": "takeoff", "params":{}}, {"action": "flip", "params":{"direction": forward}]

                        Prompt: "Take off the drone and do one flip backwards."
                        Returns: [{"action": "takeoff", "params":{}}, {"action": "flip", "params":{"direction": back}]

                        Prompt: "Take off the drone and do one flip to the right."
                        Returns: [{"action": "takeoff", "params":{}}, {"action": "flip", "params":{"direction": right}]

                        Prompt: "Take off the drone and do one flip to the left."
                        Returns: [{"action": "takeoff", "params":{}}, {"action": "flip", "params":{"direction": left}]

                        Prompt: "Do two flips to the left."
                        Returns: [{"action": "flip", "params":{"direction": left}{"action": "flip", "params":{"direction": left}]
                        
                        """



        prompt = prompt+'\nprompt: '+text_command

        # Create the message structure for the GPT-3 model
        messages = [
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": prompt}
        ]

        # Try to send the request to the GPT-3 model and handle any exceptions
        try:
            response = ollama.chat(
                model="mistral",
                messages=messages,
                options= {"temperature": 0.01}
            )
            print(response)
        except Exception as e:
            print(f"Error: {e}")
            return None

        """
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=messages,
            )
        except openai.error.InvalidRequestError as e:
            print(f"Error: {e}")
            return None
        except Exception as e:
            print(f"Unexpected error: {e}")
            return None
        
        
        """
        
        # Extract the GPT-3 model response from the returned JSON
        chatgpt_response = eval(response["message"]['content'])
        #print(chatgpt_response)
        # Find the start and end indices of the JSON string in the response
     
        # Extract the JSON string from the response
        json_response_dict = chatgpt_response
        #print('\n\n\njson_response_dict ',json_response_dict)
        return json.dumps({'text': chatgpt_response, 'json': json_response_dict})



    def post(self):
        """
        Handles an incoming POST request containing a text command. The method sends the text command
        to the GPT-3 model and processes the response using the process_and_publish_chatgpt_response function in a separate thread.
        
        Returns:
            dict: A dictionary containing the GPT-3 model response as a JSON string.
        """
        

        text_command = request.form['text_command']
        print ('[ROSGPT] Command received. ', text_command, '. Asking ChatGPT ...')
        # Run the speak function on a separate thread
        print('text_command:', text_command,'\n')
        threading.Thread(target=speak, args=(text_command+"Message received. Now consulting ChatGPT for a response.",)).start()
        chatgpt_response = self.askGPT(text_command)
        print ('[ROSGPT] Response received from ChatGPT. \n', str(json.loads(chatgpt_response))[:60], '...')
        print('eval(chatgpt_response)', eval(chatgpt_response))
        # Run the speak function on a separate thread
        threading.Thread(target=speak, args=("We have received a response from ChatGPT.",)).start()

        if chatgpt_response is None:
            return {'error': 'An error occurred while processing the request'}

        threading.Thread(target=process_and_publish_chatgpt_response, args=(self.chatgpt_ros2_node, text_command, chatgpt_response, True)).start()
        #print(json.loads(chatgpt_response))
        return json.loads(chatgpt_response)


@app.route('/nlp_control_plugin', methods=['POST'])
def receive_voice_command():
    data = request.get_json()
    text_command = data.get('command')
    print ('[ROSGPT] Voice command received: ', text_command)
    rosgpt_node.publish_message(text_command)
    return jsonify({"status": "Voice command received"})

@app.route('/')
def index():
    #print(os.path.join(app.root_path, 'webapp'))
    return send_from_directory(os.path.join(get_package_share_directory('rosgpt'), 'webapp'), 'index.html')

def main():
    rclpy.init(args=None)
    global rosgpt_node
    rosgpt_node = ROSGPTNode()
    api.add_resource(ROSGPTProxy, '/rosgpt', resource_class_args=(rosgpt_node,))
    app.run(debug=True, host='0.0.0.0', port=5000)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
