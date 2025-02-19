"""
This file contains all the common function which can be used in various
action sequences.
"""
import math
from action_sequencer.config.actions_default import sleep,move
from action_sequencer.config.positions import positions

def wait_for_data(env, attribute_name, timeout=None):
    trials = 0
    while getattr(env, attribute_name, None) is None:
        if trials == 10:
            env.logger.warn("""{} attribute pending...""".format(attribute_name))
        if env.is_cancel_requested():
            return False
        sleep(env, 0.1)
        trials += 1


def find_closest_plant(circles):
    closest = None
    min_dist = math.inf

    for circle in circles:
        dist = math.sqrt(circle.center.x**2 + circle.center.y**2)
        if (dist < min_dist):
            min_dist = dist
            closest = [circle.center.x, circle.center.y, circle.theta -
                       math.pi]

    return closest

def change_coordinates_frame(x_o,y_o, x_r, y_r, theta_r):
    x = x_o*math.cos(theta_r) - y_o*math.sin(theta_r) + x_r
    y = x_o*math.sin(theta_r) + y_o*math.cos(theta_r) + y_r
    return x, y

def move_to_target(env, x_o, y_o, theta_o=None, blocking=True, detection_mode = -1, obstacle_detection_distance = -1):
    wait_for_data(env, "RobotData")
    t_r = env.RobotData.theta.data 
    x_r = env.RobotData.position.x
    y_r = env.RobotData.position.y
    x_order , y_order = change_coordinates_frame(   x_o, 
                                                    y_o, 
                                                    x_r, 
                                                    y_r, 
                                                    t_r) 
    if theta_o != None:
        theta_order = t_r + theta_o
    else:
        theta_order = t_r
    go_to_pot = env.run_action(move,
                                blocking=blocking,
                                x=x_order,
                                y=y_order,
                                t=theta_order,
                                detection_mode=detection_mode,
                                obstacle_detection_distance=obstacle_detection_distance)
    return go_to_pot

def objectinpipe(ranges):
    """
    Check if there is an object in the pipe
    Tu use it in action_sequence_default.py:
        Package to add:
            from sensor_msgs.msg import LaserScan
        Subscribing to the lidar_data topic :
            "Lidar": ("bottom_lidar/scan", LaserScan, scan_lidar_callback)
        Line in code :
            utils.wait_for_data(self, "lidar_data")
            etat = utils.objectinpipe(self.lidar_data)
    20 is the number of inf when the plant is blocked in the door
    """
    nb_inf = 0
    for range in ranges[1500:1560]:
        if str(range) == 'inf':
            nb_inf += 1
        if nb_inf > 30:
            return True
    return False

def define_position(position_name, detection_mode=-1, obstacle_detection_distance=-1):
    x = positions[position_name][0]
    y = positions[position_name][1]
    t = positions[position_name][2]
    str_position = "{} {} {} {} {}".format(x, y, t, detection_mode, obstacle_detection_distance)
    return str_position