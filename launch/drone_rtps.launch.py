from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
    drone_controller = Node(
             package="drone",
             executable="drone_controller",
             name="drone_controller",
             output="screen",
             emulate_tty=True
#             parameters=[
#                {"connection_url": "udp://:14540"}
#             ]
           )
           
    takeoff_server = Node(
             package="drone",
             executable="takeoff_server",
             name="takeoff_server",
             output="screen",
             emulate_tty=True
           )
           
    land_server = Node(
             package="drone",
             executable="land_server",
             name="land_server",
             output="screen",
             emulate_tty=True
           )
                  
    odom_tf2_broadcaster = Node(
             package="drone",
             executable="odom_broadcaster",
             name="odom_broadcaster",
             output="screen",
             emulate_tty=True
           )
                
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','map','odom']          
    )

    map_odom_ned_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','1.57', '0', '3.14','map','odom_ned']          
    )
                
    ld.add_action(drone_controller)
    ld.add_action(odom_tf2_broadcaster)
    ld.add_action(takeoff_server)
    ld.add_action(land_server)
    ld.add_action(map_odom_tf)
    ld.add_action(map_odom_ned_tf)
    
    return ld
