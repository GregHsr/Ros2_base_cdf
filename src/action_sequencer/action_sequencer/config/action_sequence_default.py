from action_sequencer.action_sequencer_node import ActionSequence
from action_sequencer.config.actions_default import move, sleep, do_AX12action,\
                                                    set_barillet_pos, turbine_ctrl,\
                                                    set_pos, aspi_auto, set_vmax,\
                                                    syncLidarpos, step_barillet_pos,\
                                                    get_barillet_pos, do_shit, \
                                                    send_free, check_ennemi
from action_sequencer.config.callbacks import default_Odom_callback,\
                                              plant_pos_callback,\
                                              lidar_callback, default_obst_detect_callback, \
                                              scan_lidar_callback,\
                                              action_result_callback,\
                                              lidar_callback, default_obst_detect_callback

from action_sequencer.config.positions import positions
import action_sequencer.config.utils as utils
from cdf_msgs.msg import PicAction, RobotData, Obstacles, AxResult, LidarLoc, GoalDetection
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import math
from sensor_msgs.msg import LaserScan

from action_sequencer.config.action_get_plants import * 
from action_sequencer.config.action_get_pots import *

class DropPots(ActionSequence):
    name = "dropnpots"

    publishers = {
        "action": ("action", PicAction),
        "goal_position": ("goal_position", GoalDetection),
    }

    subscribers = {
        "Odom": ("Odom", RobotData, default_Odom_callback),
        "action_result": ("ax12_result", AxResult, action_result_callback),
    }

    def drop_n_pots(self, len_element):
        _ = self.run_action(do_AX12action, command="DOOROPEN", blocking=True)
        self.run_action(turbine_ctrl, perc=30)
        self.run_action(set_vmax, v_max=0.2, blocking=True)
        self.run_action(set_barillet_pos, blocking=True, pos=0)
        for _ in range(len_element):
            self.run_action(turbine_ctrl, perc=0)
            self.run_action(sleep, duration=0.3, blocking=True)
            self.run_action(turbine_ctrl, perc=30)
            self.run_action(step_barillet_pos)
            utils.move_to_target(self, -0.12, 0)
        self.run_action(turbine_ctrl, perc=0)

    def run(self, first_element, len_element):
        first_element = int(first_element)
        len_element = int(len_element)
        utils.wait_for_data(self, "RobotData")
        self.drop_n_pots(len_element)
        self.logger.info("Pots dropped")

class FollowEnnemi(ActionSequence):
    name = "followennemi"

    publishers = {
        "action": ("action", PicAction)
    }

    subscribers = {
        "lidarpos": ("top_lidar_position", LidarLoc, lidar_callback),
        "Odom": ("Odom", RobotData, default_Odom_callback)
    }

    def run(self):
        utils.wait_for_data(self, "lidar_pos")
        utils.wait_for_data(self, "RobotData")
        self.run_action(do_shit, blocking=True)


class CheckEnnemi(ActionSequence):
    name = "checkennemi"

    publishers = {
    }

    subscribers = {
        "lidarpos": ("top_lidar_position", LidarLoc, lidar_callback),
    }

    def run(self, x, y):
        x = float(x)
        y = float(y)
        utils.wait_for_data(self, "lidar_pos")
        ennemi_in_zone = self.run_action(check_ennemi, blocking=True, x=x, y=y)
        return "TRUE" if ennemi_in_zone.results() else "FALSE"

class MoveSolarPanel(ActionSequence):
    name = "movesolarpanel"

    publishers = {
        "action": ("action", PicAction),
        "goal_position": ("goal_position", GoalDetection),
    }

    subscribers = {
        "Odom": ("Odom", RobotData, default_Odom_callback),
        "obstacle_detected": ("obstacle_detected", Bool, default_obst_detect_callback)
    }

    def run(self):
        self.run_action(send_free, blocking=True)
        a = self.run_action(set_vmax, v_max=0.25)
        self.wait_for(a) 
        x = -2*0.3
        y = -2*0.15 # 0.14 -> 5° pour x=0.3
        utils.move_to_target(self, x, y)
        self.run_action(send_free, blocking=True)

class MoveSolarPanelYellow(ActionSequence):
    name = "movesolarpanelyellow"

    publishers = {
        "action": ("action", PicAction),
        "goal_position": ("goal_position", GoalDetection),
    }

    subscribers = {
        "Odom": ("Odom", RobotData, default_Odom_callback),
        "obstacle_detected": ("obstacle_detected", Bool, default_obst_detect_callback)
    }

    def run(self):
        self.run_action(send_free, blocking=True)
        a = self.run_action(set_vmax, v_max=0.25)
        self.wait_for(a) 
        x = 3*0.3
        y = 3*0.2 # 0.14 -> 5° pour x=0.3
        utils.move_to_target(self, x, y)
        self.run_action(send_free, blocking=True)

class GetPots(ActionSequence):
    name = "getpots"

    publishers = {
        "action": ("action", PicAction)
    }

    subscribers = {
        "Pots": ("bot_obstacle", Obstacles, plant_pos_callback),
        "Odom": ("Odom", RobotData, default_Odom_callback),
        "action_result": ("ax12_result", AxResult, action_result_callback)
    }

    def coordinates_pot_to_go(self, circles):
        # this function is used to choose the first pot to go to
        # it returns the coordinate to go to take the pot
        closest_pot, second_closest_pot, third_closest_pot = self.find_closest_pots_x_axis(circles)
        self.logger.info("The closest pot is : {}".format(closest_pot))
        self.logger.info("The second closest pot is : {}".format(second_closest_pot))
        self.logger.info("The third closest pot is : {}".format(third_closest_pot))
        if closest_pot is None:
            print("No pots found")
            return None, None
        # if only one pot is detected
        if second_closest_pot is None:
            x = closest_pot[0] - 0.13
            y = closest_pot[1]
            return x,y
        # if only two pots are detected
        elif third_closest_pot is None:
            if closest_pot[1]<0:
                if second_closest_pot[1] < closest_pot[1]:
                    x = closest_pot[0] - 0.11
                    y = closest_pot[1] + 0.06
                    return x,y
                else:
                    x = closest_pot[0] - 0.11
                    y = closest_pot[1] - 0.06
                    return x,y
            else:
                if second_closest_pot[1] < closest_pot[1]:
                    x = closest_pot[0] - 0.11
                    y = closest_pot[1] + 0.06
                    return x,y
                else:
                    x = closest_pot[0] - 0.11
                    y = closest_pot[1] - 0.06
                    return x,y
        #if three pots are detected
        else:
            #if closest pot is between the two others
            if second_closest_pot[1] < closest_pot[1] < third_closest_pot[1]:
                #if second closest pot is on the left side of closest pot
                if second_closest_pot[1]:
                    x = second_closest_pot[0] - 0.11
                    y = second_closest_pot[1] + 0.06
                    return x,y
                #if third closest pot is on the left side of closest pot
                else:
                    x = third_closest_pot[0] - 0.11
                    y = third_closest_pot[1] + 0.06
                    return x,y
            #if closest pot is on the left side of the two others
            elif closest_pot[1] < second_closest_pot[1] and closest_pot[1] < third_closest_pot[1]:
                x = closest_pot[0] - 0.11
                y = closest_pot[1] + 0.06
                return x,y
            #if closest pot is on the right side of the two others
            else:
                x = closest_pot[0] - 0.11
                y = closest_pot[1] - 0.06
                return x,y 


    def pot_to_go(self, circles):
        closest_pot, second_closest_pot, third_closest_pot = self.find_closest_pots_x_axis(circles)
        self.logger.info("The closest pot is : {}".format(closest_pot))
        self.logger.info("The second closest pot is : {}".format(second_closest_pot))
        self.logger.info("The third closest pot is : {}".format(third_closest_pot))

        delta_x = 0.12
        delta_y = 0.06

        if closest_pot is None:
            print("No pots found")
            return None, None
        # if only one pot is detected
        if second_closest_pot is None:
            y = closest_pot[1]
            x = closest_pot[0] - delta_x
            return x,y
        # if only two pots are detected
        elif third_closest_pot is None:
            #always got from right to left
            y = min(closest_pot[1], second_closest_pot[1])-delta_y
            x = closest_pot[0] - delta_x
            return x,y
        #if three pots are detected
        else:
            #always got from right to left
            y = min(closest_pot[1], second_closest_pot[1], third_closest_pot[1])-delta_y
            x = closest_pot[0] - delta_x
            return x,y

        
    def find_closest_pots_x_axis(self, circles):

        min_dist = math.inf
        second_min_dist = math.inf
        third_min_dist = math.inf

        for circle in circles:

            closest = None
            second_closest = None
            third_closest = None

            if circle.index_point in list(range(1400,1660)):
                dist = circle.center.x
                if (dist < min_dist):
                    second_min_dist = min_dist
                    min_dist = dist
                    second_closest = closest
                    closest = [circle.center.x, circle.center.y, circle.theta -
                            math.pi]
                elif (dist < second_min_dist):
                    third_min_dist = second_min_dist
                    third_closest = second_closest
                    second_min_dist = dist
                    second_closest = [circle.center.x, circle.center.y, circle.theta -
                                    math.pi]
                elif (dist < third_min_dist):
                    third_min_dist = dist
                    third_closest = [circle.center.x, circle.center.y, circle.theta -
                                    math.pi]
                else :
                    self.logger.info("WTF MAN")

        if ((second_closest is not None) and abs(second_closest[0]-closest[0])>0.025):
            second_closest = None
            if ((third_closest is not None) and abs(third_closest[0]-closest[0])<0.025):
                second_closest = third_closest
                third_closest = None
        if ((third_closest is not None) and abs(third_closest[0]-closest[0])>0.025):
            third_closest = None

        return closest, second_closest, third_closest

    def aspirate_pot(self, nbr_pots):
        #aspi precharge
        self.run_action(turbine_ctrl,perc=20)
        #wait for 100ms
        self.run_action(sleep, duration=0.1, blocking=True)
        #aspi on
        self.run_action(turbine_ctrl,perc=60)
        #wait for 800ms
        self.run_action(sleep, duration=0.8, blocking=True)
        # sleep(self, 0.8)
        #aspi maintient
        self.run_action(turbine_ctrl,perc=20)
        #turn barrillet
        barillet_pos_plante = self.run_action(set_barillet_pos, blocking=True, pos=nbr_pots)
        self.wait_for(barillet_pos_plante) 

    def take_pot(self):
        nbr_pots = 0
        while nbr_pots < 5:
            if nbr_pots == 0:
                #we first turn on the turnbine to precharge it
                self.run_action(turbine_ctrl,perc=20)
                #wait for 100ms
                self.run_action(sleep, duration=0.1, blocking=True)
                #turn on the turbine to aspirate the plant in front of the robot
                self.run_action(turbine_ctrl,perc=60)
                #wait for 800ms
                self.run_action(sleep, duration=0.8, blocking=True)
                #keep the turbine on to maintain the aspiration
                self.run_action(turbine_ctrl,perc=20)

            # We first open the gate and retreive the pots' positions at the
            # same time
            utils.wait_for_data(self, "last_plants_pos")
            # x,y = self.coordinates_pot_to_go(self.last_plants_pos)
            x_pot,y_pot = self.pot_to_go(self.last_plants_pos)
            if x_pot is None:
                return False

            open_door = self.run_action(do_AX12action, command="DOOROPEN")
            self.wait_for(open_door)

            # modify the goal position for the robot to move only in y axis
            utils.move_to_target(self, 0, y_pot)
            
            # modify the goal position for the robot to move only in x 
            utils.move_to_target(self, x_pot, 0)

            close_door = self.run_action(do_AX12action, command="DOORCLOSE")
            
            self.wait_for(close_door)
            if (self.Flags["Timeout_door"] == False): # doors are close properly
                self.aspirate_pot(nbr_pots)
                utils.move_to_target(self, -0.18, -y_pot)
                wait_lecture = self.run_action(sleep, blocking=True, duration=0.2)
                nbr_pots += 1
            else:
                self.logger.info("Timeout_door")
                self.run_action(do_AX12action, command="DOOROPEN")
                utils.move_to_target(self, -0.18, -y_pot)
                wait_lecture = self.run_action(sleep, blocking=True, duration=0.2)

    def run(self):
        self.take_pot()

class GetPotY(ActionSequence):
    name = "getpotsy"

    publishers = {
        "action": ("action", PicAction),
        "goal_position": ("goal_position", GoalDetection),

    }

    subscribers = {
        "Pots": ("bot_obstacle", Obstacles, plant_pos_callback),
        "Odom": ("Odom", RobotData, default_Odom_callback),
        "action_result": ("ax12_result", AxResult, action_result_callback)
    }

    def find_closest_pot_y_axis(self, circles):
        # this function is used to find the closest pot to the robot
        # it returns the closest pot in the y axis to the robot
        closest = None
        second_closest = None
        third_closest = None
        min_dist = math.inf
        second_min_dist = math.inf
        third_min_dist = math.inf

        for circle in circles:
            #dist = math.sqrt(circle.center.x**2 + circle.center.y**2)
            dist = circle.center.y
            if (dist < min_dist):
                second_min_dist = min_dist
                min_dist = dist
                second_closest = closest
                closest = [circle.center.x, circle.center.y, circle.theta -
                           math.pi]
            elif (dist < second_min_dist):
                third_min_dist = second_min_dist
                third_closest = second_closest
                second_min_dist = dist
                second_closest = [circle.center.x, circle.center.y, circle.theta -
                                 math.pi]
            elif (dist < third_min_dist):
                third_min_dist = dist
                third_closest = [circle.center.x, circle.center.y, circle.theta -
                                 math.pi]
            else :
                self.logger.info("WTF MAN")
        return closest, second_closest, third_closest

    def coordinates_pot_to_go(self, circles):
        # this function is used to choose the first pot to go to
        # it returns the coordinate to go to take the pot
        closest_pot,second_closest_pot,third_closest_pot = self.find_closest_pot_y_axis(circles)
        self.logger.info("The closest pot is : {}".format(closest_pot))
        self.logger.info("The second closest pot is : {}".format(second_closest_pot))
        self.logger.info("The third closest pot is : {}".format(third_closest_pot))
        if closest_pot is None:
            print("No pots found")
            return None, None
        x = closest_pot[0] - 0.15
        y = closest_pot[1] - 0.06
        return x,y
    
    def take_pot(self):
        nbr_pots = 0
        while nbr_pots < 5:
            # We first open the gate and retreive the pots' positions at the
            # same time
            utils.wait_for_data(self, "last_plants_pos")
            print("The pots' positions are : {}".format(self.last_plants_pos))
            x,y = self.coordinates_pot_to_go(self.last_plants_pos)
            if x is None:
                return False

            open_door = self.run_action(do_AX12action, command="DOOROPEN")

            self.wait_for(open_door)

            # modify the goal position for the robot to move only in y axis
            x_o, y_o = utils.change_coordinates_frame(0,
                                                      y,
                                                      self.RobotData.theta.data)
            x_o = 0
            y_o = y
            go_to_pot = self.run_action(move,
                                        blocking=True,
                                        x=x_o+self.RobotData.position.x,
                                        y=y_o+self.RobotData.position.y)
            
            # modify the goal position for the robot to move only in x axis
            x_o, y_o = utils.change_coordinates_frame(x, 
                                                      0,
                                                      self.RobotData.theta.data)
            x_o = x
            y_o = 0
            go_to_pot = self.run_action(move,
                                        blocking=True,
                                        x=x_o + self.RobotData.position.x,
                                        y=y_o + self.RobotData.position.y)

            close_door = self.run_action(do_AX12action, command="DOORCLOSE")
            self.wait_for(close_door)
            self.run_action(aspi_auto, blocking=True)
            wait_aspi = self.run_action(sleep, duration=0.8) 

            x_o, y_o = utils.change_coordinates_frame(-0.2, 
                                                      0,
                                                      self.RobotData.theta.data)
            x_o = -0.1
            y_o = 0
            wait_lecture = self.run_action(sleep, blocking=True, duration=0.2)
            self.run_action(move,
                            blocking=True,
                            x=x_o+self.RobotData.position.x,
                            y=y_o+self.RobotData.position.y)
            
            self.wait_for(wait_aspi)
            #self.run_action(move,
            #                blocking=True,
            #                x=self.RobotData.position.x,
            #                y=self.RobotData.position.y-0.1)
            nbr_pots += 1

    def run(self):
        self.run_action(set_pos, x=0,
                                y=0,
                                t=0)
        self.take_pot()

class LidarSync(ActionSequence):
    name = "lidarsync"

    publishers = {
        "action": ("action", PicAction)
    }

    subscribers = {
        "lidarpos": ("top_lidar_position", LidarLoc, lidar_callback)
    }

    def run(self):
        utils.wait_for_data(self, "lidar_pos")
        self.run_action(syncLidarpos, blocking=True)

class SetVmax(ActionSequence):
    name = "setvmax"

    publishers = {
        "action": ("action", PicAction)
    }

    subscribers = {}

    def run(self, v_max):
        a = self.run_action(set_vmax, v_max=v_max, blocking=True)

class SendFree(ActionSequence):
    name = "sendfree"

    publishers = {
        "action": ("action", PicAction)
    }
    subscribers = {}
    def run(self):
        self.run_action(send_free, blocking=True)


class SetPos(ActionSequence):
    name = "setpos"

    publishers = {
        "action": ("action", PicAction)
    }

    subscribers = {}

    def run(self, x, y, theta):
        self.run_action(set_pos, x=x, y=y, t=theta, blocking=True)

class SimpleActionSeq(ActionSequence):
    name = "simpleactionseq"

    publishers = {
        "action": ("action", PicAction)
    }

    subscribers = {
        "action_result": ("ax12_result", AxResult, action_result_callback)
    }

    def run(self):
        a = self.run_action(do_AX12action, command="DOOROPEN")
        self.run_action(turbine_ctrl, perc=25)
        self.wait_for(a)
        self.run_action(turbine_ctrl, perc=-25)
        a = self.run_action(do_AX12action, command="DOORCLOSE")
        zzz = self.run_action(sleep, duration=2)
        self.wait_for(a, zzz)
        self.run_action(set_barillet_pos, pos=3)

class DoAxAction(ActionSequence):
    name = "doaxaction"

    publishers = {
        "action": ("action", PicAction)
    }

    subscribers = {
        "action_result": ("ax12_result", AxResult, action_result_callback)
    }

    def run(self, action_name, ignore_done=False):
        a = self.run_action(do_AX12action, command=action_name, ignore_done=ignore_done)
        self.wait_for(a)

class Move(ActionSequence):
    name = "move"

    publishers = {
        "action": ("action", PicAction),
        "goal_position": ("goal_position", GoalDetection),
    }

    subscribers = {
        "obstacle_detected": ("obstacle_detected", Bool, default_obst_detect_callback),
        "Odom": ("Odom", RobotData, default_Odom_callback)
    }

    def run(self, x, y, t, detection_mode=-1, obstacle_detection_distance=-1, end_distance=0.02):
        x = float(x)
        y = float(y)
        detection_mode = int(detection_mode)
        obstacle_detection_distance = float(obstacle_detection_distance)
        end_distance = float(end_distance)
        if t is not None:
            t = float(t)
        a = self.run_action(move, x=x, y=y, t=t, detection_mode=detection_mode, obstacle_detection_distance=obstacle_detection_distance, end_distance=end_distance)
        self.wait_for(a)

"""
class DropPot(ActionSequence):
    name = "droppot"
    publishers = {
        "action": ("action", PicAction)
    }

    subscribers = {
        "action_result": ("ax12_result", AxResult, action_result_callback)
    }

    def run(self,index):
        a = self.run_action(do_AX12action, command="DOOROPEN")
        self.wait_for(a)
        self.run_action(set_barillet_pos, pos=index)
"""
