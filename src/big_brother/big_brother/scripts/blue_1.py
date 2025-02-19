from big_brother.big_brother_seq import Script
from action_sequencer.config.positions import positions
import action_sequencer.config.utils as utils
import time

from math import pi

start_pos = [1.86, 0.145, 4.18]
front_of_plant_island = [1.70, 1.03, pi]
pt4 = [1.3875, 0.350, 3*pi/2]
pt5 = [1.3875, 0.200, 3*pi/2]
pt6 = [1.750, 0.365, 0]
pt_inter = [0.5, 0.5, 5 * pi / 4]
pt7 = [0.15, 0.15, 5 * pi / 4]


class Default_script(Script):
    def __init__(self, parent_node):
        super().__init__(parent_node)

    def run(self):
        begin = time.time()
        self.dropnpots_started = False
        self.doActionSeq("lidarsync")

        self.doActionSeq("move", "{} {} {} {} -1".format(start_pos[0],
                                                     start_pos[1],
                                                     start_pos[2], -1)
                         )
        self.doActionSeq("movesolarpanel")
        self.add_score(15)
        self.doActionSeq("lidarsync")
        self.doActionSeq("move", "{} {} {} {} -1".format(front_of_plant_island[0],
                                                   front_of_plant_island[1],
                                                   front_of_plant_island[2], -1)
                         )
        self.doActionSeq("lidarsync")
        self.doActionSeq("move", "{} {} {} {} -1".format(front_of_plant_island[0],
                                                   front_of_plant_island[1],
                                                   front_of_plant_island[2], -1)
                         )
        time.sleep(0.1)
        res = self.doActionSeq("getplantbis")

        poussette = False
        if res[1].context.data.split()[0] == "abort":
            poussette = True
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {} -1".format(1.6,
                                                   1.25,
                                                   pi, -1)
                         )
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {} -1".format(1.1,
                                                    1.15,
                                                    5.36, -1) #34*pi/24
                             )
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {} -1".format(1.1,
                                                    1.15,
                                                    5.36, -1)
                             )
            self.doActionSeq("doaxaction", "DOOROPEN")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {} -1".format(1.65,
                                                    0.35,
                                                    5.36, -1)
                             )
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "1.4 0.6 5.36 -1 -1")
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
            self.doActionSeq("move", "{} {} {} {} -1".format(1.6,
                                                   1.25,
                                                   pi, -1)
                         )
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {} -1".format(1.1,
                                                    1.15,
                                                    5.36, -1) #34*pi/24
                             )
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {} -1".format(1.1,
                                                    1.15,
                                                    5.36, -1)
                             )
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {} -1".format(1.65,
                                                    0.35,
                                                    5.36, -1)
                             )
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "1.4 0.6 5.36 -1 -1")

        self.doActionSeq("lidarsync")
        self.doActionSeq("move", utils.define_position("P1_front"))
        self.doActionSeq("lidarsync")
        self.doActionSeq("move", utils.define_position("P1_front"))
        self.doActionSeq("getpotbis", str(liste))
        time.sleep(0.5)
        self.doActionSeq("lidarsync")
        time.sleep(0.5)
        self.doActionSeq("lidarsync")
        self.doActionSeq("move", "{} {} {} {} -1".format(pt_inter[0],
                                                   pt_inter[1],
                                                   pt_inter[2], -1)
                         )
        self.doActionSeq("lidarsync")                    
        self.doActionSeq("move", "{} {} {} {} -1".format(pt7[0],
                                                   pt7[1],
                                                   pt7[2], -1)
                         )
        self.doActionSeq("lidarsync")
        self.doActionSeq("move", "{} {} {} {} -1".format(pt7[0],
                                                   pt7[1],
                                                   pt7[2], -1)
                         )
        self.doActionSeq("lidarsync")
        self.dropnpots_started = True
        self.add_score(10)
        self.doActionSeq("dropnpots", f"{0} {3}")
        self.doActionSeq("lidarsync")    
        ennemi_in_zone = self.doActionSeq("checkennemi", "0.7 2.6")
        if time.time() - begin < 83 and ennemi_in_zone[1].context.data == "FALSE":
            self.add_score(-10)
            self.doActionSeq("move", f"0.4 0.6 {pi / 2 - 0.2} -1 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", f"0.4 0.6 {pi / 2 - 0.2} -1 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("sendfree")
            self.doActionSeq("setvmax", "0.9")
            self.doActionSeq("move", "{} {} {} {} 0.75 0.1".format(0.8,
                                                    2.7,
                                                    pi / 2 - 0.2, -1))
            self.add_score(10)
            self.add_score(3)
            self.doActionSeq("sendfree")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {} -1".format(0.8,
                                                    2.7,
                                                    pi / 2 - 0.2, -1))
            self.doActionSeq("lidarsync")            
            self.doActionSeq("move", "{} {} {} {} -1".format(0.64,
                                                    2.8,
                                                    0, -1))
            self.doActionSeq("lidarsync")            
            self.doActionSeq("move", "{} {} {} {} -1".format(0.64,
                                                    2.8,
                                                    0, -1))
            
        else:               
            self.doActionSeq("move", "{} {} {} {} -1".format(0.45,
                                                       0.2,
                                                       2.5, -1))
            self.doActionSeq("lidarsync")  
            self.doActionSeq("move", "{} {} {} {} -1".format(0.45,
                                                       0.2,
                                                       2.5, -1))