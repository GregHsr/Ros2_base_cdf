from action_sequencer.action_sequencer_node import ActionSequence
from action_sequencer.config.actions_default import move, sleep, do_AX12action,\
                                                    set_barillet_pos, turbine_ctrl,\
                                                    set_pos, aspi_auto, set_vmax,\
                                                    syncLidarpos, step_barillet_pos,\
                                                    get_barillet_pos
from action_sequencer.config.callbacks import default_Odom_callback,\
                                              plant_pos_callback,\
                                              lidar_callback, default_obst_detect_callback, \
                                              scan_lidar_callback,\
                                              action_result_callback,\
                                              lidar_callback, default_obst_detect_callback

from action_sequencer.config.positions import positions
import action_sequencer.config.utils as utils
from cdf_msgs.msg import PicAction, RobotData, Obstacles, AxResult, LidarLoc, GoalDetection
from std_msgs.msg import Bool
import math
from sensor_msgs.msg import LaserScan

class GetPotBis(ActionSequence):
    name = "getpotbis"

    publishers = {
        "action": ("action", PicAction),
        "goal_position": ("goal_position", GoalDetection),
    }

    subscribers = {
        "Pots": ("bot_obstacle", Obstacles, plant_pos_callback),
        "Odom": ("Odom", RobotData, default_Odom_callback),
        "action_result": ("ax12_result", AxResult, action_result_callback),
        "Lidar": ("bottom_lidar/scan", LaserScan, scan_lidar_callback),
    }

    def pot_to_go(self, circles):
        closest_plant, second_closest_plant, third_closest_plant = self.find_closest_plants_x_axis(circles)

        delta_x = 0.135
        delta_y = 0.06

        if closest_plant is None:
            self.logger.info("No pot detected")
            return None, None
        # if only one plant is detected
        if second_closest_plant is None:
            y = closest_plant[1]
            x = closest_plant[0] - delta_x
            return x,y
        # if only two plant are detected
        elif third_closest_plant is None:
            #always got from right to left
            y = min(closest_plant[1], second_closest_plant[1])-delta_y
            x = closest_plant[0] - delta_x
            return x,y
        #if three plant are detected
        else:
            #always got from right to left
            y = min(closest_plant[1], second_closest_plant[1], third_closest_plant[1])-delta_y
            x = closest_plant[0] - delta_x
            return x,y

    def pot_to_go_bis(self, circles):
        closest_plant, second_closest_plant, third_closest_plant = self.find_closest_plants_x_axis(circles)
        delta_x = 0.13
        delta_y = 0.06

        if closest_plant is None:
            self.logger.info("No pot detected")
            return None, None
        # if only one plant is detected
        if second_closest_plant is None:
            y = closest_plant[1]
            x = closest_plant[0] - delta_x
            return x,y
        # if only two plant are detected
        elif third_closest_plant is None:
            #always got from right to left
            y = max(closest_plant[1], second_closest_plant[1]) + delta_y
            x = closest_plant[0] - delta_x
            return x,y
        #if three plant are detected
        else:
            #always got from right to left
            y = min(closest_plant[1], second_closest_plant[1], third_closest_plant[1])-delta_y
            x = closest_plant[0] - delta_x
            return x,y

    def find_closest_plants_x_axis(self, circles):
        closest = None
        second_closest = None
        third_closest = None
        min_dist = math.inf
        second_min_dist = math.inf
        third_min_dist = math.inf

        for circle in circles:
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

    def _move_to_plant_and_grab(self, x, y):
        max_iter = 2
        _ = self.run_action(do_AX12action, command="DOOROPEN", blocking=True)
        utils.move_to_target(self, 0, y)
        utils.move_to_target(self, x, 0)
        is_closed = self.run_action(do_AX12action, command="DOORCLOSE")
        self.wait_for(is_closed)
        i = 0
        while not is_closed.results() and i < max_iter:
            _ = self.run_action(do_AX12action, command="DOOROPEN", blocking=True)
            self.logger.info(f"1 - Door not closed correctly, retrying {i+1}/{max_iter}")
            utils.move_to_target(self, -0.025, 0)
            utils.move_to_target(self, 0.015, 0)
            is_closed = self.run_action(do_AX12action, command="DOORCLOSE")
            self.wait_for(is_closed)
            i += 1
        if not is_closed.results():
            self.logger.warn("1 - Door not closed correctly")
            return False
        return True

    def _aspirate(self, up=False):
        self.run_action(turbine_ctrl,perc=10)
        self.run_action(sleep, duration=0.05, blocking=True)
        self.run_action(turbine_ctrl,perc=90)
        self.run_action(sleep, blocking=True, duration=0.5)
        self.run_action(turbine_ctrl,perc=50)

    def take_pots(self):
        # Secu to know if there are many plants in the pipe
        close_door = self.run_action(do_AX12action, command="DOORCLOSE", blocking=True)
        if not close_door.results():
            self.logger.warn("1 - Traffic jam or door blocked")
            return 'trafficjam'
        self.logger.info("1 - Door closed")
        self.run_action(turbine_ctrl,perc=0)
        self.run_action(sleep, blocking=True, duration=0.1)

        # Count the nunmber of True in the pipeList
        nbr_plant = sum(self.pipeList)
        self.logger.info(f"1 - Number of plant detected: {nbr_plant}")

        # We just iterate through the pipeList and take the pots that have a plant
        self.run_action(set_barillet_pos, blocking=True, pos=0)
        for i, plant in enumerate(self.pipeList):
            # If there are no more plants in the pipe, we stop
            if sum(self.pipeList[i:]) == 0:
                break
            if plant == False:
                self._aspirate()
                self.run_action(step_barillet_pos, blocking=True)
                continue
            else:
                self.logger.info("1 - Plant detected")
                is_closed = self.run_action(do_AX12action, command="DOORCLOSE")
                self.wait_for(is_closed)
                self._aspirate()
                self.run_action(sleep, blocking=True, duration=0.2)
                utils.wait_for_data(self, "last_plants_pos")
                if i == 3:
                    x_pot,y_pot = self.pot_to_go_bis(self.last_plants_pos)
                else:
                    x_pot,y_pot = self.pot_to_go(self.last_plants_pos)
                if x_pot is None:
                    return False
                if not self._move_to_plant_and_grab(x_pot, y_pot):
                    self.logger.info("1 - Not grabbed")
                    self.run_action(turbine_ctrl,perc=0)
                    return False
                self.logger.info("1 - Aspirating both")
                if i != nbr_plant-1:
                    self._aspirate(False)
                    a = self.run_action(step_barillet_pos)
                    b = utils.move_to_target(self, -0.075, -y_pot)
                    self.wait_for(a, b)
                    if not a.results()[0]:
                        self._aspirate(False)
                        a = self.run_action(step_barillet_pos)
                        if not a.results()[0]:
                            self.run_action(turbine_ctrl,perc=0)
                            return False
        self.run_action(turbine_ctrl,perc=0)         

    def run(self, pipeList="[True, True, True]"):
        try:
            self.pipeList = eval(pipeList)
        except:
            self.pipeList = [True, True, True]
        if sum(self.pipeList) == 0:
            return 'no_plant'
        result = self.take_pots()
        return result
