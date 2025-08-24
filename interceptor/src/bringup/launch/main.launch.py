from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    cdriver_node = Node(
        package="cdriver",
        executable="image_offline_node"
    )
    
    detection_node = Node(
        package="yolov8",
        executable="detection_yolov8_node"
    )
     
    # rdriver_node = Node(
    #     package="rdriver",
    #     executable="listener"
    # )
    
    ld.add_action(cdriver_node)
    ld.add_action(detection_node)
    # ld.add_action(rdriver_node)
    
    return ld