import os
import json
import math
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory
import ollama

class AIBrainNode(Node):
    def __init__(self):
        super().__init__('ai_brain')
        self.get_logger().info("Initializing AI Brain Node...")

        # Load known semantic locations from config
        config_path = os.path.join(
            get_package_share_directory('embodied_assistant'),
            'config',
            'semantic_locations.yaml'
        )
        with open(config_path, 'r') as f:
            self.locations = yaml.safe_load(f)['locations']
            
        self.available_locations = list(self.locations.keys())
        self.get_logger().info(f"Loaded locations for LLM Context: {self.available_locations}")

        # Initialize Navigation Commander
        self.navigator = BasicNavigator()

    def yaw_to_quaternion(self, yaw):
        return {
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)
        }

    def execute_navigation(self, location_name: str) -> bool:
        if location_name not in self.locations:
            self.get_logger().error(f"LLM hallucinated location '{location_name}'!")
            return False

        loc_data = self.locations[location_name]
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = float(loc_data['x'])
        goal_pose.pose.position.y = float(loc_data['y'])
        
        quat = self.yaw_to_quaternion(float(loc_data['yaw']))
        goal_pose.pose.orientation.z = quat['z']
        goal_pose.pose.orientation.w = quat['w']

        self.navigator.goToPose(goal_pose)
        
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()
        return result == TaskResult.SUCCEEDED

    def process_user_command(self, user_prompt: str):
        # Construct strict system prompt
        system_prompt = f"""
You are an autonomous AI Robot Assistant.
Your task is to analyze user requests and translate them into a single valid action.

Available locations in your memory: {self.available_locations}

STRICT OUTPUT FORMAT RULES:
Return ONLY a valid JSON object matching this schema without any markdown wrapping or extra text:
{{
    "thought": "Your step-by-step reasoning here",
    "action": "navigate" OR "speak_only",
    "target_location": "one of the available locations OR null",
    "spoken_response": "What the robot should say to the user explaining its action"
}}
"""

        self.get_logger().info(f"Processing command with LLM: '{user_prompt}'")

        try:
            response = ollama.chat(
                model='llama3.2',
                messages=[
                    {'role': 'system', 'content': system_prompt},
                    {'role': 'user', 'content': user_prompt}
                ],
                format='json'
            )
            
            result_json = json.loads(response['message']['content'])
            
            print("\n================ AI COGNITIVE REASONING ================")
            print(f"Thought Process : {result_json.get('thought')}")
            print(f"Action Selected : {result_json.get('action')}")
            print(f"Target Location : {result_json.get('target_location')}")
            print(f"Robot Speech    : \"{result_json.get('spoken_response')}\"")
            print("=======================================================\n")

            # Execute action
            action = result_json.get('action')
            target = result_json.get('target_location')

            if action == 'navigate' and target in self.locations:
                self.get_logger().info(f"Executing motion to '{target}'...")
                success = self.execute_navigation(target)
                if success:
                    self.get_logger().info(f"Successfully arrived at {target}!")
                else:
                    self.get_logger().warn("Navigation failed or was canceled.")
            else:
                self.get_logger().info("No navigation required for this command.")

        except Exception as e:
            self.get_logger().error(f"Error communicating with LLM Brain: {e}")

def main(args=None):
    rclpy.init(args=args)
    brain = AIBrainNode()
    
    while rclpy.ok():
        try:
            user_input = input("\nTalk to Robot Assistant (or type 'exit'): ")
            if user_input.lower().strip() == 'exit':
                break
            if user_input.strip():
                brain.process_user_command(user_input)
        except KeyboardInterrupt:
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()
