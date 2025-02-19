import math
import time
import rclpy
from cdf_msgs.msg import PicAction, RobotData, GoalDetection
from rclpy.wait_for_message import wait_for_message
from geometry_msgs.msg import Point


def sleep(env, duration):
    start_time = time.time()

    while rclpy.ok():
        if env.is_cancel_requested():
            return False
        if time.time() - start_time >= duration:
            return True
        time.sleep(0.1)


def syncLidarpos(env):
    # Now we can get the position and set the robot
    robot_pos = env.lidar_pos.robot_position

    msg = PicAction()
    msg.action_destination  = "CAN"
    msg.action_msg = "free"
    env.publishers['action'].publish(msg)

    set_pos(env, robot_pos.x, robot_pos.y, robot_pos.z)

def do_shit(env):
    begin = time.time()
    # while time.time() - begin < 300:
    while True:
        ennemis = env.lidar_pos.other_robot_position
        robot = env.lidar_pos.robot_position
        env.logger.info(f"Position Robot {robot}")
        env.logger.info(f"Position Ennemi {ennemis}")
        dst = 1000000
        closest = None
        for ennemi in ennemis:
            dst_test = (ennemi.x - robot.x) **2 + (ennemi.y - robot.y) **2
            if dst_test < dst:
                closest = ennemi
                dst = dst_test
        if closest is None:
            env.logger.info(f"Found")
            msg = PicAction()
            msg.action_destination  = "CAN"
            msg.action_msg = "free"
            env.publishers['action'].publish(msg)
        else:
            env.logger.info(f"Not found")
            msg = PicAction()
            msg.action_destination  = "CAN"
            msg.action_msg = f"move {ennemi.x} {ennemi.y} {env.RobotData.theta.data}"
            env.publishers['action'].publish(msg)
            time.sleep(0.1)

def check_ennemi(env, x, y):
    ennemis = env.lidar_pos.other_robot_position
    env.logger.info(f"Position Ennemi {ennemis}")
    for ennemi in ennemis:
        if (ennemi.x - x)**2 + (ennemi.y - y)**2 < 0.45:
            return True
    return False

def do_AX12action(env, command, ignore_done=False):
    env.logger.info("doing AX12Action: %s" % command)
    msg = PicAction()
    msg.action_destination = "SERVO"
    msg.action_msg = command
    env.publishers['action'].publish(msg)
    env.Flags = dict()
    env.Flags["Timeout_door"] = False

    if getattr(env, "action_results", True):
        env.action_results = dict()

    env.action_results[command] = None

    if ignore_done:
        return True

    while rclpy.ok():
        if env.is_cancel_requested():
            del env.action_results[command]
            return False
        if env.action_results[command] == 0:
            env.logger.info("AX12Action %s done" % command)
            del env.action_results[command]
            return True
        elif env.action_results[command] == 1:
            env.logger.info("AX12Action %s Timeout" % command)
            env.Flags["Timeout_door"] = False
            del env.action_results[command]
            return False
        elif env.action_results[command] == 2:
            env.logger.info("AX12Action %s Busy" % command)
            del env.action_results[command]
            sleep(env, 0.5)
            do_AX12action(env, command)
        elif env.action_results[command] == 6:
            env.logger.info("AX12Action %s Timeout_open_door" % command)
            del env.action_results[command]
            env.Flags["Timeout_door"] = True
            return False
        
        sleep(env, 0.05)


def turbine_ctrl(env, perc):
    if perc < -100 or perc > 100:
        env.logger.error("Incorrect percent value (-100:100%)")
        return False
    msg = PicAction()
    msg.action_destination = 'USER'
    msg.action_msg = "turbine %i" % perc
    env.publishers['action'].publish(msg)
    return True


def set_barillet_pos(env, pos):
    pos = int(pos)
    if pos < 0 or pos > 4:
        env.logger.error("Incorrect barrel index (must be >= 0 and <= 4)")
        return False
    msg = PicAction()
    msg.action_destination = 'USER'
    msg.action_msg = "barillet %i" % pos
    env.publishers['action'].publish(msg)

    # If the dictionary doesn't exist, we create it
    if getattr(env, "action_results", True):
        env.action_results = dict()

    # We then initialize the result of the action
    env.action_results["BARILLET_POS"] = None

    while rclpy.ok():
        if env.is_cancel_requested():
            del env.action_results["BARILLET_POS"]
            return (False, -1)
        elif env.action_results["BARILLET_POS"] is None:
            continue
        elif env.action_results["BARILLET_POS"] == -1:
            env.logger.error("Barrel Totaly blocked")
            del env.action_results["BARILLET_POS"]
            return (False, -1)
        elif env.action_results["BARILLET_POS"] == pos:
            env.logger.info("Barrel reached pos %i" % pos)
            del env.action_results["BARILLET_POS"]
            return (True, pos)
        elif env.action_results["BARILLET_POS"] >= 10:
            env.logger.error("Barrel failed to reach pos %i" % pos)
            env.logger.error("It is now stopped at pos %i" % (env.action_results["BARILLET_POS"] - 10))
            stopped_pos = env.action_results["BARILLET_POS"]
            del env.action_results["BARILLET_POS"]
            return (False, stopped_pos - 10)
        
        sleep(env, 0.05)

def get_barillet_pos(env):
    msg = PicAction()
    msg.action_destination = 'USER'
    msg.action_msg = "barillet"
    env.publishers['action'].publish(msg)

    # If the dictionary doesn't exist, we create it
    if getattr(env, "action_results", True):
        env.action_results = dict()

    # We then initialize the result of the action
    env.action_results["BARILLET_POS"] = None

    while rclpy.ok():
        if env.is_cancel_requested():
            del env.action_results["BARILLET_POS"]
            return False
        if env.action_results["BARILLET_POS"] is not None:
            env.logger.info("Barrel is at pos %i" % env.action_results["BARILLET_POS"])
            stopped_pos = env.action_results["BARILLET_POS"]
            del env.action_results["BARILLET_POS"]
            return (True, stopped_pos)
        
        sleep(env, 0.05)

def step_barillet_pos(env, Clockwise=True):
    msg = PicAction()
    msg.action_destination = 'USER'
    msg.action_msg = "B%s" % ("P" if Clockwise else "M")
    env.publishers['action'].publish(msg)

    # If the dictionary doesn't exist, we create it
    if getattr(env, "action_results", True):
        env.action_results = dict()

    # We then initialize the result of the action
    env.action_results["BARILLET_POS"] = None

    while rclpy.ok():
        if env.is_cancel_requested():
            del env.action_results["BARILLET_POS"]
            return (False, -1)
        elif env.action_results["BARILLET_POS"] is None:
            continue
        elif env.action_results["BARILLET_POS"] == -1:
            env.logger.error("Barrel Totaly blocked")
            del env.action_results["BARILLET_POS"]
            return (False, -1)
        elif env.action_results["BARILLET_POS"] < 10:
            env.logger.info("Barrel reached pos %i" % env.action_results["BARILLET_POS"])
            stopped_pos = env.action_results["BARILLET_POS"]
            del env.action_results["BARILLET_POS"]
            return (True, stopped_pos)
        elif env.action_results["BARILLET_POS"] is not None:
            env.logger.error("Barrel failed to update its position")
            env.logger.error("It is now stopped at pos %i" % (env.action_results["BARILLET_POS"] - 10))
            stopped_pos = env.action_results["BARILLET_POS"]
            del env.action_results["BARILLET_POS"]
            return (False, stopped_pos - 10)
        
        sleep(env, 0.05)

def check_tof_plant(env):
    msg = PicAction()
    msg.action_destination = 'SERVO'
    msg.action_msg = "TOFBARILLET"
    env.publishers['action'].publish(msg)

    # If the dictionary doesn't exist, we create it
    if getattr(env, "action_results", True):
        env.action_results = dict()

    # We then initialize the result of the action
    env.action_results["TOFBARILLET"] = None

    while rclpy.ok():
        if env.is_cancel_requested():
            del env.action_results["TOFBARILLET"]
            return False
        if env.action_results["TOFBARILLET"] is not None:
            env.logger.info("TOF plant is %s the pipe" % ("in" if env.action_results["TOFBARILLET"] else "out"))
            in_pipe = env.action_results["TOFBARILLET"]
            del env.action_results["TOFBARILLET"]
            env.logger.info(f"in pipe: {in_pipe}")
            return in_pipe
        
        sleep(env, 0.05)

def aspi_auto(env):
    msg = PicAction()
    msg.action_destination = 'USER'
    msg.action_msg = "aspi"
    env.publishers['action'].publish(msg)
    return True


def set_pos(env, x=None, y=None, t=None):
    msg = PicAction()
    msg.action_destination = 'CAN'

    if x is not None:
        msg.action_msg = "setx {}".format(str(x))
        env.publishers['action'].publish(msg)
    if y is not None:
        msg.action_msg = "sety {}".format(str(y))
        env.publishers['action'].publish(msg)
    if t is not None:
        msg.action_msg = "sett {}".format(str(t))
        env.publishers['action'].publish(msg)

    return True

def set_vmax(env, v_max=None):
    msg = PicAction()
    msg.action_destination = 'CAN'

    if v_max is not None:
        msg.action_msg = "vmax {}".format(str(v_max))
        env.publishers['action'].publish(msg)

    return True

def send_free(env):
    msg = PicAction()
    msg.action_destination = 'CAN'

    msg.action_msg = "free"
    env.publishers['action'].publish(msg)

    return True

def move(env, x=0, y=0, t=None, detection_mode=-1, obstacle_detection_distance= -1, end_distance=0.02):
    trials = 0
    msg = PicAction()
    msg.action_destination = 'CAN'
    env.obstacle_detected = False

    while getattr(env, "RobotData", None) is None:
        if trials == 10:
            env.logger.warn("""The Robot seems not sending its position""")
        if env.is_cancel_requested():
            msg.action_msg = "free"
            env.publishers['action'].publish(msg)
            return False
        sleep(env, 0.1)
        trials += 1
    if t is None:
        t = env.RobotData.theta.data

    msg.action_msg = "move {} {} {}".format(x, y, t)
    env.publishers['action'].publish(msg)
    goal_detection = GoalDetection()
    goal_detection.detection_mode = detection_mode
    goal_detection.goal_position = Point(x=float(x), y=float(y), z=float(t))
    goal_detection.obstacle_detection_distance = float(obstacle_detection_distance)
    env.publishers['goal_position'].publish(goal_detection)
    distance = math.inf
    while True:
        if env.is_cancel_requested():
            msg.action_msg = "free"
            env.publishers['action'].publish(msg)
            return False
        if env.obstacle_detected:
            env.logger.warn("Obstacle detected, letting avoid_obstacle handle it")
            while env.obstacle_detected:
                if env.is_cancel_requested():
                    msg.action_msg = "free"
                    env.publishers['action'].publish(msg)
                    return False
                sleep(env, 0.05)
            # syncLidarpos(env)
            env.logger.warn("Out of the object, going to the goal back")
            env.publishers['action'].publish(msg)

        elif env.RobotData is not None:
            distance = math.sqrt((env.RobotData.position.x - x)**2 +
                                 (env.RobotData.position.y - y)**2)
        else:
            distance = math.inf
        if distance < end_distance:
            break
        sleep(env, 0.01)
    goal_detection = GoalDetection()
    goal_detection.detection_mode = -1
    goal_detection.goal_position = Point(x=float(-1), y=float(-1), z=float(-1))
    goal_detection.obstacle_detection_distance = float(-1)
    env.publishers['goal_position'].publish(goal_detection)
    return True
