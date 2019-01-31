#!/usr/bin/python3.6
#
# Copyright 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

"""Python launch file for median_filter_5"""

def generate_launch_description():
    parameters_file_dir = pathlib.Path(__file__).resolve().parent
    parameters_file_path = parameters_file_dir / 'median_filter_5.yaml'
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    os.environ['TOPIC_NAME'] = 'scan'
    topic_prefix = 'base_'

    median_filter_5 = launch_ros.actions.Node(
            package='laser_filters', node_executable='scan_to_scan_filter_chain',
            node_name='scan_filter_chain',
            output='screen',
            remappings=[
            ('scan', 'base_scan'),
            (EnvironmentVariable(name='TOPIC_NAME'), [
                topic_prefix, EnvironmentVariable(name='TOPIC_NAME')])
            ],
            parameters=[
                parameters_file_path,
                str(parameters_file_path),
                [EnvironmentVariable(name='FILE_PATH'), os.sep, 'median_filter_5.yaml'],
            ],
        )

    return LaunchDescription([
        median_filter_5,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=median_filter_5,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )),
])    
