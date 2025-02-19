from big_brother.big_brother_seq import Script
from action_sequencer.config.positions import positions
import action_sequencer.config.utils as utils
import time

from math import pi

start_pos = [1.86, 3-0.145, 4.18]
front_of_plant_island = [1.70, 2.03, pi]
pt4 = [1.3875, 0.350, 3*pi/2]
pt5 = [1.3875, 0.200, 3*pi/2]
pt6 = [1.750, 0.365, 0]
pt_inter = [0.5, 2.5, 3 * pi / 2] # 5 * pi / 4 + pi
pt7 = [0.15, 2.85, 11 * pi / 4] # 5 * pi / 4 + 3 * pi / 2 (270°)


class Default_script(Script):
    def __init__(self, parent_node):
        self.endofmatch_time = 200
        super().__init__(parent_node)

    def run(self):
        begin = time.time()
        self.dropnpots_started = False
        self.doActionSeq("lidarsync")

        self.doActionSeq("move", "{} {} {} {}".format(start_pos[0],
                                                     start_pos[1],
                                                     start_pos[2], -1)
                         )
        self.doActionSeq("movesolarpanelyellow")
        self.add_score(15)

        self.doActionSeq("lidarsync")
        self.doActionSeq("move", f"1.75 2 {start_pos[2]} -1")
        self.doActionSeq("lidarsync")
        self.doActionSeq("move", f"1.75 2 {start_pos[2]} -1")
        self.doActionSeq("lidarsync")
        self.doActionSeq("move", "{} {} {} {}".format(front_of_plant_island[0],
                                                   front_of_plant_island[1],
                                                   front_of_plant_island[2], -1)
                         )
        self.doActionSeq("lidarsync")
        self.doActionSeq("move", "{} {} {} {}".format(front_of_plant_island[0],
                                                   front_of_plant_island[1],
                                                   front_of_plant_island[2], -1)
                         )
        time.sleep(0.1)
        res = self.doActionSeq("getplantbis")

        poussette = False
        if res[1].context.data.split()[0] == "abort":
            poussette = True 
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {}".format(1.6,
                                                   1.75,
                                                   pi, -1)
                         )
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {}".format(1.1,
                                                    1.65,
                                                    1.05, -1)
                             )
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {}".format(1.1,
                                                    1.65,
                                                    1.05, -1)
                             )
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {}".format(1.65,
                                                    2.65,
                                                    1.05, -1)
                             )
            self.doActionSeq("move", utils.define_position("I5_center"))
            self.add_score(4)

        elif res[1].context.data.split()[0] == "done":
            liste = res[1].context.data.split()[1:]
            liste = "".join(liste)
            liste = eval(liste)
            for i in range (len(res)):
                if res[i]==True:
                    self.add_score(4)

        if poussette != True :
            self.doActionSeq("doaxaction", 'DOOROPEN')
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {}".format(1.6,
                                                   1.75,
                                                   pi, -1)
                         )
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {}".format(1.1,
                                                    1.65,
                                                    1.05, -1)
                             )
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {}".format(1.1,
                                                    1.65,
                                                    1.05, -1)
                             )
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {}".format(1.65,
                                                    2.65,
                                                    1.05, -1)
                             )
            self.doActionSeq("move", utils.define_position("I5_center"))

        self.doActionSeq("lidarsync")
        self.doActionSeq("move", utils.define_position("P6_front"))
        self.doActionSeq("lidarsync")
        self.doActionSeq("move", utils.define_position("P6_front"))

        # self.doActionSeq("lidarsync")
        # self.doActionSeq("move", "{} {} {} {}".format(pt4[0],
        #                                            pt4[1],
        #                                            pt4[2], -1)
        #                  )
        # self.doActionSeq("lidarsync")
        # self.doActionSeq("move", "{} {} {} {}".format(pt4[0],
        #                                            pt4[1],
        #                                            pt4[2], -1)
        #                  )
        self.doActionSeq("getpotbis")
        time.sleep(0.5)
        self.doActionSeq("lidarsync")
        time.sleep(0.5)
        self.doActionSeq("lidarsync")
        self.doActionSeq("move", "{} {} {} {}".format(pt_inter[0],
                                                   pt_inter[1],
                                                   pt_inter[2], -1)
                         )
        self.doActionSeq("lidarsync")
        self.doActionSeq("move", "{} {} {} {}".format(pt_inter[0],
                                                   pt_inter[1],
                                                   pt_inter[2], -1)
                         )
        time.sleep(0.5)
        self.doActionSeq("lidarsync")                         
        self.doActionSeq("move", "{} {} {} {}".format(pt7[0],
                                                   pt7[1],
                                                   pt7[2], -1)
                         )
        self.dropnpots_started = True
        self.add_score(10)
        self.doActionSeq("dropnpots", f"{0} {3}")
        self.doActionSeq("lidarsync")    
        if time.time() - begin < 83:      
            self.doActionSeq("sendfree")
            self.doActionSeq("setvmax", "0.8")

            self.doActionSeq("move", "{} {} {} {} 0.75".format(1.0,
                                                    0.4,
                                                    4.14, -1))
            self.doActionSeq("sendfree")
            self.doActionSeq("lidarsync")                         
            self.doActionSeq("move", "{} {} {} {} -1".format(1.0,
                                                    0.4,
                                                    1.0, -1))
            self.doActionSeq("lidarsync")        
            self.doActionSeq("move", "{} {} {} {} -1".format(1.3,
                                                    0.2,
                                                    1.0, -1))
        else:               
            self.doActionSeq("move", "{} {} {} {} -1".format(0.45,
                                                       2.8,
                                                       5.14, -1))


    def pre_endofmatch_handler(self):
        if not self.dropnpots_started:
            self.doActionSeq("lidarsync", endofmatch=True)
            self.doActionSeq("move", "{} {} {} {}".format(pt7[0],
                                                   pt7[1],
                                                   pt7[2], -1)
            )
            self.add_score(10)
            self.doActionSeq("dropnpots", f"{0} {3}")
