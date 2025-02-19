from big_brother.big_brother_seq import Script

class Default_script(Script):
    def run(self):
        self.doActionSeq("lidarsync")
        self.doActionSeq("followennemi")