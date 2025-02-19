from big_brother.big_brother_seq import Script
from action_sequencer.config.positions import positions
import action_sequencer.config.utils as utils
import time

from math import pi

class Default_script(Script):
    def __init__(self, parent_node):
        self.endofmatch_time = 200
        super().__init__(parent_node)

    def run(self):
        for _ in range(30):
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "1.5 0.5 0 -1 0.25")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "1.5 0.5 0 -1 0.25")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "1.5 2.5 0 -1 0.75")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "1.5 2.5 0 -1 0.75")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "1.5 1.5 0 -1 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "1.5 1.5 0 -1 -1")
    
    def pre_endofmatch_handler(self):
        self.doActionSeq("lidarsync", endofmatch=True)
        self.doActionSeq("move", "{} {} {} -1".format(1, 1.5 , pi), endofmatch=True)
