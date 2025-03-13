# #!/usr/bin/env python3

# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description():
#     # ดึง path ของ launch และ world file
#     launch_file_dir = os.path.join(get_package_share_directory('leader_bot'), 'launch')
#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

#     use_sim_time = LaunchConfiguration('use_sim_time', default='true')
#     x_pose = LaunchConfiguration('x_pose', default='-2.0')
#     y_pose = LaunchConfiguration('y_pose', default='-0.5')

#     world = os.path.join(
#         get_package_share_directory('leader_bot'),
#         'worlds',
#         'turtlebot3_world.world'
#     )

#     # สั่งเปิด Gazebo Server พร้อมโหลด world
#     gzserver_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
#         ),
#         launch_arguments={'world': world}.items()
#     )

#     # สั่งเปิด Gazebo GUI
#     gzclient_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
#         )
#     )

#     # เปลี่ยนจาก SDF เป็นเรียกใช้ spawn_turtlebot3.launch.py ที่ใช้ URDF
#     spawn_turtlebot_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
#         ),
#         launch_arguments={
#             'x_pose': x_pose,
#             'y_pose': y_pose
#         }.items()
#     )

#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{'use_sim_time': True}],
#         output='screen'
#     )


#     # เพิ่มทั้งหมดลงใน LaunchDescription
#     ld = LaunchDescription()
#     ld.add_action(gzserver_cmd)
#     ld.add_action(gzclient_cmd)
#     ld.add_action(spawn_turtlebot_cmd)
#     ld.add_action(robot_state_publisher)

#     return ld

#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro  

def generate_launch_description():
    # Package และ Path ต่าง ๆ (ไม่เปลี่ยน path world และ spawn)
    package_name = "leader_bot"
    world_file_name = "turtlebot3_world.world"
    
    models_directory = os.path.join(
        get_package_share_directory(package_name), "models"
    )
    world_file_path = os.path.join(
        get_package_share_directory(package_name), "worlds", world_file_name
    )
    robot_description_file = os.path.join(
        get_package_share_directory(package_name), "urdf", "turtlebot3_burger_cam.urdf"
    )
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), "rviz", "config_rviz.rviz"
    )

    # ตรวจสอบว่าไฟล์ world และ robot urdf มีอยู่จริง
    if not os.path.exists(world_file_path):
        raise FileNotFoundError(f"World file not found: {world_file_path}")
    if not os.path.exists(robot_description_file):
        raise FileNotFoundError(f"Robot description file not found: {robot_description_file}")

    # อ่านไฟล์ URDF
    with open(robot_description_file, "r") as urdf_file:
        robot_description = urdf_file.read()

    # กำหนด GAZEBO_MODEL_PATH เพื่อให้ Gazebo รู้จักโมเดล
    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=models_directory,
    )

    # เรียกใช้งาน Gazebo และกำหนดให้ใช้ world ที่ระบุ
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={"world": world_file_path, "gui": "true", "use_sim_time": "true"}.items(),
    )

    # Robot State Publisher สำหรับ Broadcast URDF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # Spawn หุ่นยนต์ใน Gazebo (ไม่เปลี่ยน path และ parameter)
    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "robot",
            "-file", robot_description_file,
            "-x", "0", "-y", "0", "-z", "0",
            "-R", "0", "-P", "0", "-Y", "0"
        ],
        output="screen",
    )

    # เปิด RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
    )

    # Controller Manager สำหรับควบคุมหุ่นยนต์
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Spawner สำหรับ Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Spawner สำหรับ Robot Velocity Controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Static TF publisher (world -> odom)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_footprint"],
        output="screen"
        
    )
    

    # Odometry Publisher สำหรับการส่งค่า TF
    odom_publisher = Node(
        package="leader_bot",
        executable="tf_publisher.py",
        name="tf_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # Delay Spawner เพื่อให้ Controller Manager พร้อมก่อน
    delay_controller_spawners = TimerAction(
        period=5.0,  # ดีเลย์ 5 วินาที
        actions=[joint_state_broadcaster_spawner, robot_controller_spawner],
    )

    # Joint Trajectory Controller Spawner
    joint_trajectory_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )


    # Return Launch Description
    return LaunchDescription([   
        set_gazebo_model_path,
        gazebo,
        spawn_robot_node,
        robot_state_publisher,
        rviz_node,
        # controller_manager,
        # delay_controller_spawners,
        static_tf,
        odom_publisher,
        joint_state_publisher,
        # joint_trajectory_position_controller_spawner
    ])
