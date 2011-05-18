import subprocess as sb
import os

class CmdProcess:
    
    def __init__(self, cmd_list):
        self.cmd_list= cmd_list
        self.process = None

    def run(self):
        self.process = sb.Popen(self.cmd_list)

    def kill(self):
        os.system('kill -2 %d' % self.process.pid)

    def is_finished( self ):
        return self.process.poll() != None

