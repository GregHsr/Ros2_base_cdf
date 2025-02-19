from action_sequencer.action_sequencer_node import ActionSequence
from action_sequencer.config.actions_default import move, sleep, do_AX12action,\
                                                    set_barillet_pos, turbine_ctrl,\
                                                    set_pos, aspi_auto, set_vmax,\
                                                    syncLidarpos, step_barillet_pos,\
                                                    get_barillet_pos, check_tof_plant
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


class GetPlantBis(ActionSequence):
    name = "getplantbis"

    publishers = {
        "action": ("action", PicAction),
        "goal_position": ("goal_position", GoalDetection),
    }

    subscribers = {
        "Pots": ("bot_obstacle", Obstacles, plant_pos_callback),
        "Odom": ("Odom", RobotData, default_Odom_callback),
        "Lidar": ("bottom_lidar/scan", LaserScan, scan_lidar_callback),
        "action_result": ("ax12_result", AxResult, action_result_callback),
        "obstacle_detected": ("obstacle_detected", Bool, default_obst_detect_callback),
    }

    def plant_to_go(self, circles):
        closest_plant, second_closest_plant, third_closest_plant = self.find_closest_plants_x_axis(circles)
        self.logger.info("The closest plant is : {}".format(closest_plant))
        self.logger.info("The second closest plant is : {}".format(second_closest_plant))
        self.logger.info("The third closest plant is : {}".format(third_closest_plant))

        delta_x = 0.115
        delta_y = 0.055

        if closest_plant is None:
            print("No plants found")
            return None, None
        # if only one plant is detected
        if second_closest_plant is None:
            y = closest_plant[1]
            x = closest_plant[0] - delta_x
            return x,y
        # if only two plant are detected
        elif third_closest_plant is None:
            # always got from right to left
            y = min(closest_plant[1], second_closest_plant[1])-delta_y
            x = closest_plant[0] - delta_x
            return x,y
        # if three plant are detected
        else:
            # always got from right to left
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

    def aspirate_plant(self, fail_mode=False):
        # #aspi precharge
        # self.run_action(turbine_ctrl,perc=20)
        # #wait for 100ms
        # self.run_action(sleep, duration=0.05, blocking=True)

        # aspi on
        self.run_action(turbine_ctrl,perc=90)
        # wait for 800ms
        if (fail_mode == False):
            self.run_action(sleep, duration=0.5, blocking=True)
        else:
            self.run_action(sleep, duration=1.5, blocking=True)
        # aspi maintient
        self.run_action(turbine_ctrl,perc=30)
        self.run_action(sleep, duration=0.02, blocking=True)

        # turn barrillet
        barillet_pos_plante = self.run_action(step_barillet_pos, blocking=True)
        self.run_action(sleep, duration=0.05, blocking=True)
        self.run_action(turbine_ctrl,perc=5)
        self.run_action(sleep, duration=0.05, blocking=True)
        self.run_action(turbine_ctrl,perc=0)
        sleep(self, 0.1)

        if not barillet_pos_plante.results()[0]:
            self.logger.warn("Barillet not turned")
            return False, False

        check_in_pipe_res = self.run_action(check_tof_plant, blocking=True)

        if not check_in_pipe_res.results():
            self.run_action(sleep, duration=0.1, blocking=True)
            check_in_pipe_res = self.run_action(check_tof_plant, blocking=True)

        if not check_in_pipe_res.results():
            self.logger.warn("Plant not in pipe")
            return True, False

        return True, True

    def abort_this_plant(self):
        self.run_action(turbine_ctrl,perc=-90)
        self.run_action(sleep, duration=0.5, blocking=True)

    def handle_door_timeout(self):
        utils.move_to_target(self, -0.04, 0, blocking=True, detection_mode=1, obstacle_detection_distance=0.25)
        self.run_action(do_AX12action, command="DOOROPEN", blocking=True)
        utils.move_to_target(self, 0.03, 0, detection_mode=1, obstacle_detection_distance=0.25)
        close_door = self.run_action(do_AX12action, command="DOORCLOSE", blocking=True)
        second_timeout = close_door.results()
        if not second_timeout:
            utils.move_to_target(self, -0.03, 0, blocking=True, detection_mode=1, obstacle_detection_distance=0.25)
            close_door = self.run_action(do_AX12action, command="DOOROPEN", blocking=True)
            utils.move_to_target(self, 0.02, 0, blocking=True, detection_mode=1, obstacle_detection_distance=0.25)
            close_door = self.run_action(do_AX12action, command="DOORCLOSE", blocking=True)
            third_timeout = close_door.results()
            if third_timeout == False :
                self.run_action(do_AX12action, command="DOOROPEN", blocking=True)
                self.run_action(turbine_ctrl,perc=0)
                return False
        return True

    def handle_plantnotinpipe(self,x,y):
        self.tentative_abort += 1
        self.run_action(do_AX12action, command="DOOROPEN")
        utils.move_to_target(self, -x + 0.10, -y, blocking = True, detection_mode=1, obstacle_detection_distance=0.25)
        if self.tentative_abort == 3:
            return False

    def take2_plants(self):
        nbr_plants = 0
        fail_nb = 0
        barrel_state = [False,False,False,False,False]
        self.run_action(sleep, blocking=True, duration=0.4)
        while nbr_plants < 3 and fail_nb < 3:
            self.logger.info(f"Plant number {nbr_plants}")
            utils.wait_for_data(self, "last_plants_pos")
            x_plant, y_plant = self.plant_to_go(self.last_plants_pos)
            if x_plant is None:
                return f'abort {barrel_state}'

            self.run_action(turbine_ctrl,perc=80)
            self.run_action(do_AX12action, command="DOOROPEN", blocking=True)

            utils.move_to_target(self, 0, y_plant, detection_mode=1, obstacle_detection_distance=0.25)
            utils.move_to_target(self, x_plant, 0, detection_mode=1, obstacle_detection_distance=0.25)
            close_door = self.run_action(do_AX12action, command="DOORCLOSE", blocking=True)
            timeout_door = close_door.results()
            utils.move_to_target(self, -0.05, 0, blocking=True, detection_mode=1, obstacle_detection_distance=0.25)

            if not timeout_door:
                self.logger.warn("Door were not closed")
                result = self.handle_door_timeout()
                if not result:
                    return f'abort {barrel_state}' # la on abandonne c'est mort la plante est couché

            utils.move_to_target(self, -0.05, -y_plant, blocking=False, detection_mode=1, obstacle_detection_distance=0.25)
            if nbr_plants != 4: #on met 4 ici dans le cas ou joue avec 5 plante, la 5e (nbr 4) n'a pas à être aspiré de suite 
                barillet, aspiration = self.aspirate_plant()

                if barillet and aspiration:
                    barrel_state[nbr_plants] = True
                    nbr_plants += 1
                elif not barillet:
                    self.logger.warn("Plant not aspirated")
                    # a = self.run_action(step_barillet_pos, Clockwise=False)
                    self.run_action(do_AX12action, command="DOOROPEN", blocking=True)
                    utils.move_to_target(self, 0.01, 0, blocking=False, detection_mode=1, obstacle_detection_distance=0.25)
                    self.run_action(do_AX12action, command="DOORCLOSE", blocking=True)
                    # self.wait_for(a)
                    barillet, aspiration = self.aspirate_plant(fail_mode=True)

                    if barillet and aspiration:
                        barrel_state[nbr_plants] = True
                        nbr_plants += 1
                    else:
                        return f'abort {barrel_state}'
                else:
                    self.logger.warn("Plant not in pipe")
                    a = self.run_action(step_barillet_pos, Clockwise=False)
                    utils.move_to_target(self, -0.04, 0, blocking=False, detection_mode=1, obstacle_detection_distance=0.20)
                    self.wait_for(a)
                    fail_nb += 1
                    continue
            else :
                nbr_plants += 1

        self.run_action(turbine_ctrl,perc=0)
        return f'done {barrel_state}'

    def run(self):
        self.tentative_abort = 0
        result = self.take2_plants()
        return result
