from big_brother.big_brother_seq import Script
import action_sequencer.config.utils as utils
import time

from math import pi

class Default_script(Script):
    def __init__(self, parent_node):
        self.endofmatch_time = 15
        super().__init__(parent_node)

    def run(self):
        for _ in range(30):
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "1.65 0.35 0 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "1.65 0.35 0 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "1.65 2.65 0 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "1.65 2.65 0 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "0.35 2.65 0 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "0.35 2.65 0 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "0.35 0.35 0 -1")
            self.doActionSeq("lidarsync")
            self.doActionSeq("move", "0.35 0.35 0 -1")

    def pre_endofmatch_handler(self):
        self.doActionSeq("lidarsync", endofmatch=True)
        self.doActionSeq("move", "{} {} {} -1".format(1, 1.5 , pi), endofmatch=True)
