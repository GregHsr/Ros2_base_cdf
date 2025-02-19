"""
This file contains all the callbacks used in all the action sequences.

Each of the callbacks have to take a first positional argument called env,
which represents the node currently used to run the current action sequence.
The second mandatory positionnal argument contains the message we want to
process
"""


def action_result_callback(env, msg):
    env.action_results[msg.action_name] = msg.result
    env.logger.info("Action result received: {} {}".format(msg.action_name, msg.result))

def default_Odom_callback(env, msg):
    env.RobotData = msg


def plant_pos_callback(env, msg):
    env.last_plants_pos = msg.circles


def lidar_callback(env, msg):
    env.lidar_pos = msg


def default_obst_detect_callback(env, msg):
    env.obstacle_detected = msg.data


def scan_lidar_callback(env, msg):
    env.lidar_data = msg.ranges
