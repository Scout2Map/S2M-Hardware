import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg_share = get_package_share_directory('s2m_description')
    world_file = os.path.join(pkg_share, 'worlds', 'slip_test.world.sdf')

    # heightmap image is referenced by relative uri, gz needs the worlds dir on its resource path
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        additional_env={'GZ_SIM_RESOURCE_PATH': os.path.join(pkg_share, 'worlds')},
        output='screen',
    )

    return LaunchDescription([gz_sim])
