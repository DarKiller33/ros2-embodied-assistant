import os
import math
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory

class SemanticNavigator(Node):
    def __init__(self):
        super().__init__('semantic_navigator')
        self.get_logger().info("Initializing Semantic Navigator Node...")
        
        # Load Semantic Map Config
        config_path = os.path.join(
            get_package_share_directory('embodied_assistant'),
            'config',
            'semantic_locations.yaml'
        )
        
        with open(config_path, 'r') as f:
            self.locations = yaml.safe_load(f)['locations']
            
        self.get_logger().info(f"Loaded semantic locations: {list(self.locations.keys())}")
        
        # Nav2 Simple Commander Client
        self.navigator = BasicNavigator()
        self.get_logger().info("Semantic Navigator is ready!")

    def yaw_to_quaternion(self, yaw):
        return {
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)
        }

    def navigate_to_location(self, location_name: str) -> bool:
        location_name = location_name.lower().strip()
        
        if location_name not in self.locations:
            self.get_logger().error(f"Location '{location_name}' not found in semantic map!")
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

        self.get_logger().info(f"Navigating to '{location_name}' at ({loc_data['x']}, {loc_data['y']})...")
        
        self.navigator.goToPose(goal_pose)
        
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Successfully reached target: '{location_name}'")
            return True
        else:
            self.get_logger().warn(f"Failed to reach '{location_name}'. Reason code: {result}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavigator()
    
    print("\n--- Available Locations ---")
    for key in node.locations.keys():
        print(f" - {key}")
        
    target = input("\nEnter location name to navigate: ")
    node.navigate_to_location(target)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
