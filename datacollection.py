from klampt import vectorops, se3, so3
import cPickle as pickle
import time, re, os

class dataCollector:
    "takes two world models (object and hand) and computes/collects the data"
    def __init__(self, worldModel, objectModel, robotModel):
        self.world = worldModel
        self.object = objectModel
        self.robot = robotModel

        self.objXform = None
        self.robotXform = None
        self.relativeXform = None
        self.robotConfig = []

        # store configuration of these links
        self.validLinks = [8,9,13,14,18]
        swivel_finger_links = [8,13]
        proximal_finger_links = [9,14,18]

    def update(self):
        self.objXform = self.object.getTransform()
        self.robotXform = self.robot.link(6).getTransform()
        self.relativeXform = se3.mul(se3.inv(self.objXform), self.robotXform)

        # for sanity check (checked that the relative transform output is correct)
        # print "obj", self.objXform[1], "robot", self.robotXform[1], "relative", self.relativeXform[1]

        self.robotConfig = []
        for i in self.validLinks:
            self.robotConfig.append(self.robot.getConfig()[i])
        # print self.robotConfig

    def newFile(self, printout = True):
        timestr = time.strftime("%Y%m%d-%H%M%S")
        fileName = "data_logs/datalog_"+timestr+".txt"
        f = open(fileName, "w")
        f.write("")
        f.close()

        if printout:
            print "created a new log file! (", fileName, ")"

    def save(self, printout = True):
        r = re.compile(r'datalog_\d{8}-\d{6}\.txt$')
        fileName = "data_logs/"+max(filter(r.search,os.listdir('data_logs/')))

        logEntry = str([self.relativeXform, self.robotConfig])
        logEntry = logEntry.translate(None, '([]),')

        f = open(fileName, "a")
        f.write( logEntry + '\n')
        f.close()

        if printout:
            print "current relative xform and config saved! (", fileName, ")"
