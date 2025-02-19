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
        self.doActionSeq("move", "1.86 2.855 4.18 -1 -1")
        self.doActionSeq("movesolarpanelyellow")
        self.add_score(15)
        self.doActionSeq("lidarsync")
        self.doActionSeq("setvmax", "0.6")
        self.doActionSeq("move", f"1.2 2.5 4.18 -1 -1")
        self.doActionSeq("lidarsync")
        self.doActionSeq("move", f"0.95 2.02 {pi} -1 -1")
        self.doActionSeq("lidarsync")
        self.doActionSeq("move", f"0.95 2.02 {pi} -1 -1")
        res = self.doActionSeq("getplantbis")
        liste = res[1].context.data.split()[1:]
        liste = "".join(liste)
        liste = eval(liste)
        cp_liste = liste.copy()
        self.doActionSeq("lidarsync")
        self.doActionSeq("move", "1.1 1.6 0.92 -1 -1")
        self.doActionSeq("lidarsync")
        self.doActionSeq("move", "1.1 1.6 0.92 -1 -1")
        self.doActionSeq("doaxaction", "DOOROPEN")
        self.doActionSeq("lidarsync")
        self.doActionSeq("move", "1.65 2.65 0.92 -1 -1")
        self.add_score(3)
        self.doActionSeq("lidarsync")
        self.doActionSeq("move", "1.6 2.4 0.92 -1 -1")
        self.doActionSeq("lidarsync")
        if liste != [False, False, False, False, False]:
            self.doActionSeq("move", utils.define_position("P6_front"))
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", utils.define_position("P6_front"))
            self.logger.info(f"liste0: {liste}")
            liste = res[1].context.data.split()[1:]
            self.logger.info(f"liste1: {liste}")
            liste = "".join(liste)
            self.logger.info(f"liste2: {liste}")
            self.doActionSeq("getpotbis", str(liste))
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", f"0.35 2.65 {3 * pi / 4} -1 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", f"0.17 2.83 {3 * pi / 4} -1 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", f"0.17 2.83 {3 * pi / 4} -1 -1")
            self.dropnpots_started = True
            self.add_score(10)
            self.doActionSeq("dropnpots", f"{0} {3}")
            for i in range (len(cp_liste)):
                if cp_liste[i]:
                    self.add_score(4)
                    self.logger.info(f"Score add 4 thx to {i}")
            self.doActionSeq("lidarsync")    
        else:
            self.logger.info(f"No barells abort")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", f"0.45 2.55 {3 * pi / 2 + 0.2} -1 -1")
            self.add_score(10)
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", f"0.45 2.55 {3 * pi / 2 + 0.2} -1 -1")
        ennemi_in_zone = self.doActionSeq("checkennemi", "0.7 0.4")
        if time.time() - begin < 83 and ennemi_in_zone[1].context.data == "FALSE":
            self.add_score(-10)
            self.doActionSeq("move", f"0.4 2.4 {3 * pi / 2 + 0.2} -1 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", f"0.4 2.4 {3 * pi / 2 + 0.2} -1 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("sendfree")
            self.doActionSeq("setvmax", "0.9")
            self.doActionSeq("move", f"0.8 0.3 {3 * pi / 2 + 0.2} -1 0.75 0.1")
            self.add_score(10)
            self.add_score(3)
            self.doActionSeq("sendfree")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", f"0.8 0.3 {3 * pi / 2 + 0.2} -1 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "0.64 0.2 0 -1 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "0.64 0.2 0 -1 -1")
        else:
            self.doActionSeq("move", "0.45 2.8 3.78 -1 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "0.45 2.8 3.78 -1 -1")
