from big_brother.big_brother_seq import Script
import action_sequencer.config.utils as utils
import time

from math import pi

pt7 = [0.17, 0.17, 5 * pi / 4]

class Default_script(Script):
    def __init__(self, parent_node):
        super().__init__(parent_node)

    def run(self, ennemis_pos, lidar_pos):
        begin = time.time()
        ennemis = ennemis_pos
        robot = lidar_pos
        dst = 1000000
        closest = None
        for ennemi in ennemis:
            dst_test = (ennemi.x - robot.x) **2 + (ennemi.y - robot.y) **2
            if dst_test < dst:
                closest = ennemi
                dst = dst_test
        x = closest.x
        y = closest.y
        if x > 1.0 and y > 1.5:
            pos_ennemi_init = 'left'
        elif x < 1.0 and y > 1.5:
            pos_ennemi_init = 'right'
        else:
            pos_ennemi_init = 'front'
        self.logger.info(f"Position ennemi init: {pos_ennemi_init}")
        if pos_ennemi_init == 'left':
            self.doActionSeq("setvmax", "1.5")
            self.doActionSeq("move", f"1.7 1.0 0.9 -1 0.75")
            self.doActionSeq("lidarsync")
            self.doActionSeq("setvmax", "0.7")
            self.doActionSeq("move", f"1.8 0.45 0.9 -1 0.75")
            self.add_score(6)
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {} -1".format(1.65,
                                                     0.4,
                                                     4.18, -1))
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {} -1".format(1.75,
                                                     0.3,
                                                     4.18, -1))
            self.doActionSeq("lidarsync")
            self.doActionSeq("setvmax", "0.3")
            self.doActionSeq("move", "{} {} {} {} -1".format(1.86, 
                                                     0.145, 
                                                     4.18, -1))
            self.doActionSeq("move", "{} {} {} {} -1".format(1.91, 
                                                     1.0, 
                                                     4.18, -1))
            self.add_score(15)
            res = self.doActionSeq("getennemiposition")
            x = float(res[1].context.data.split()[0])
            y = float(res[1].context.data.split()[1])
            if x < 1.5 and (y > 2 or y < 1):
                self.doActionSeq("lidarsync")
                self.doActionSeq("sendfree")
                self.doActionSeq("setvmax", "0.3")
                self.doActionSeq("move", "{} {} {} {} -1".format(1.91, 
                                                     1.9, 
                                                     4.18, -1))
                self.add_score(15)
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", f"0.95 0.97 {pi} -1 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", f" 0.95 0.97 {pi} -1 -1")
            res = self.doActionSeq("getplantbis")
            liste = res[1].context.data.split()[1:]
            liste = "".join(liste)
            liste = eval(liste)
            cp_liste = liste.copy()
            self.logger.info(f"Liste: {liste}")
            if liste != [False, False, False, False, False]:
                self.doActionSeq("move", utils.define_position("P1_front"))
                self.doActionSeq("lidarsync")
                self.doActionSeq("move", utils.define_position("P1_front"))
                liste = res[1].context.data.split()[1:]
                liste = "".join(liste)
                self.doActionSeq("getpotbis", str(liste))
                self.doActionSeq("lidarsync")
                self.doActionSeq("move", f"0.4 0.4 {5 * pi / 4} -1 -1")
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
                self.dropnpots_started = True
                self.doActionSeq("dropnpots", f"{0} {3}")
                self.score += 10
                for i in range (len(cp_liste)):
                    if cp_liste[i]:
                        self.score += 4
                        self.logger.info(f"Score add 4 thx to {i} and list is {liste[i]}")
                self.doActionSeq("lidarsync")
            else :
                self.logger.info(f"No barells abort")
                self.doActionSeq("lidarsync")
                self.doActionSeq("move", "0.45 0.45 5.2 -1 -1")
                self.doActionSeq("lidarsync")
                self.doActionSeq("move", "0.45 0.45 5.2 -1 -1")
            self.add_score(10)
            ennemi_in_zone = self.doActionSeq("checkennemi", "0.7 2.6")
            if time.time() - begin < 83 and ennemi_in_zone[1].context.data == "FALSE":
                self.score -= 10
                self.doActionSeq("move", f"0.4 0.6 {pi / 2 - 0.2} -1 -1")
                time.sleep(0.1)
                self.doActionSeq("lidarsync")
                self.doActionSeq("move", f"0.4 0.6 {pi / 2 - 0.2} -1 -1")
                time.sleep(0.1)
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
                                                        0.3,
                                                        2.5, -1))
                self.doActionSeq("lidarsync")  
                self.doActionSeq("move", "{} {} {} {} -1".format(0.45,
                                                        0.3,
                                                        2.5, -1))
                












        elif pos_ennemi_init == 'right':
            self.doActionSeq("setvmax", "1.5")
            self.doActionSeq("move", f"0.4 1.0 0.1 -1 0.75")
            self.doActionSeq("lidarsync")
            self.doActionSeq("setvmax", "0.7")
            self.doActionSeq("move", f"0.35 0.35 0.1 -1 0.75")
            self.add_score(6)
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", f"1.05 1.25 5.2 -1 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", f"1.05 1.25 5.2 -1 -1")
            self.doActionSeq("doaxaction", "DOOROPEN")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {} -1".format(1.65,
                                                    0.35,
                                                    5.2, -1)
                                )
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "{} {} {} {} -1".format(1.75,
                                                    0.3,
                                                    4.18, -1))
            self.doActionSeq("lidarsync")
            self.doActionSeq("setvmax", "0.3")
            self.doActionSeq("move", "{} {} {} {} -1".format(1.86, 
                                                    0.145, 
                                                    4.18, -1))
            self.doActionSeq("move", "{} {} {} {} -1".format(1.91, 
                                                    1.0, 
                                                    4.18, -1))
            self.add_score(15)
            res = self.doActionSeq("getennemiposition")
            x = float(res[1].context.data.split()[0])
            y = float(res[1].context.data.split()[1])
            if x < 1.5 and (y > 2 or y < 1):
                self.doActionSeq("lidarsync")
                self.doActionSeq("sendfree")
                self.doActionSeq("setvmax", "0.3")
                self.doActionSeq("move", "{} {} {} {} -1".format(1.91, 
                                                    1.9, 
                                                    4.18, -1))
                self.add_score(15)
                self.doActionSeq("lidarsync")
                self.doActionSeq("move", f"1.2 2.4 1.04 -1 -1")
                self.doActionSeq("lidarsync")
                self.doActionSeq("move", f"1.2 2.4 1.04 -1 -1")
            else:
                self.doActionSeq("lidarsync")
                self.doActionSeq("move", f"1.7 1.1 1.04 -1 -1")
                self.doActionSeq("lidarsync")
            self.doActionSeq("move", f"1.75 0.3 1.04 -1 -1")
        elif pos_ennemi_init == 'front':
            self.doActionSeq("setvmax", "1.5")
            self.doActionSeq("move", f"1.0 1.5 0.5 -1 0.75")
            self.doActionSeq("lidarsync")
            self.doActionSeq("setvmax", "0.7")
