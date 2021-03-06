import math, random
from klampt import vectorops, se3, so3, loader, gldraw, ik, contact,robotcollide
import planner
from operator import itemgetter
import copy

# Joint numbers
swivel_finger_links = [8,13]
proximal_finger_links = [9,14,18]
base_links = [0,1,2,3,4,5]

class PoseGenerator:
    def __init__(self, world, robotModel, robotController):
        self.world = world
        self.robot = robotModel
        self.controller = robotController
        self.planner = planner.LimbPlanner(world)

        self.base_range = [0.005,0.005,0.005]
        self.pose_range = [math.pi/180,math.pi/180,math.pi/180]

    def collisionFree(self, config):
        self.planner = planner.LimbPlanner(self.world)
        return self.planner.check_collision_free(config)

    def randomPose(self, qcmd = None, range = 1.0):
        qmin,qmax = self.robot.getJointLimits()
        if qcmd == None:
            center = self.controller.getCommandedConfig()
        else:
            center = qcmd
        q = center
        rangeVal = self.base_range+self.pose_range

        for j in base_links:
            # sampling from uniform distribution
            # minimum = max(qmin[j],center[j]-rangeVal[j]*range)
            # maximum = min(qmax[j],center[j]+rangeVal[j]*range)
            # q[j] = random.uniform(minimum, maximum)

            # sampling from normal distribution
            minimum = qmin[j]
            maximum = qmax[j]
            q[j] = min(max(random.gauss(center[j], rangeVal[j]*range),minimum), maximum)
        return q

    # TODO: incrementally further away from the INITIAL config, not the latest random config!!!
    def randomPoses(self, iter, range=None):
        initialConfig = self.controller.getCommandedConfig()
        qList = []
        # for i in range(iter):
        i = 1
        while len(qList) < iter:
            self.robot.setConfig(initialConfig)

            # q = self.randomPose(qcmd = copy.copy(initialConfig), range=i/5.0)
            if range == None:
                # print "range is not specified. Use ", i/1.0
                q = self.randomPose(qcmd = copy.copy(initialConfig), range=i/1.0)
            else:
                # print "range specified as", range
                q = self.randomPose(qcmd = copy.copy(initialConfig), range=range)

            # if randomPose is in collision with world, create another one
            if not self.collisionFree(q):
                continue

            self.robot.setConfig(q)
            d = self.distance()

            # hand too close to object
            dObjHand = d[0]
            if dObjHand < 0.0001:
                continue

            # hand too close to ground
            dGroundHand = d[1]
            if dGroundHand < 0.01:
                continue

            # print i, d
            qList.append(q)
            i += 1

        return qList

    def distance(self):
        obj = self.world.rigidObject(0).geometry()
        ground = self.world.terrain(0).geometry()

        # distance between hand links and object
        d = []
        for i in range(self.robot.numLinks()):
            link = self.robot.link(i).geometry()
            d.append(obj.distance(link))
        dObjHand = min(d[5:])

        # distance between hand links and ground
        d = []
        for i in range(self.robot.numLinks()):
            link = self.robot.link(i).geometry()
            d.append(ground.distance(link))
        dGroundHand = min(d[5:])

        # distance between ground and object
        dGroundObj = ground.distance(obj)

        return [dObjHand, dGroundHand, dGroundObj]

    def resetWorld(self):
        self.world.rigidObject(0).geometry().translate([1,1,1])


    def get_ik_solutions(self,goal,maxResults=10,maxIters=1000,tol=1e-3,printer=True):
        initialConfig = self.controller.getCommandedConfig()
        validity_checker = self.planner.check_collision_free

        numTrials = 0
        ikSolutions = []
        numSolutions = 0
        numColFreeSolutions = 0

        for i in range(1):
            numTrials += 1
            # if first time trying the ik goal, initialize with current config
            if numTrials == 1:
                self.robot.setConfig(initialConfig)
            # else, initialize with a random q, incrementally perturbing more from inital config
            else:
                self.robot.setConfig(self.randomPose())

            #print self.robot.getConfig()
            if ik.solve(goal,tol=tol):
                numSolutions += 1
                if validity_checker():
                    numColFreeSolutions += 1
                    ikSolutions.append(self.robot.getConfig())
                    if len(ikSolutions) >= maxResults: break

        if printer:
            print "< IK Summary >",
            print "Attempted:", numTrials, "/", maxIters, "; Result:", numColFreeSolutions, "/", numSolutions, "col. free. solutions"

        # sort solutions by distance to initial config
        sortedSolutions = []
        for solution in ikSolutions:
            dist = vectorops.distanceSquared(solution,initialConfig)
            sortedSolutions.append( (dist, solution) )

        sortedSolutions = sorted(sortedSolutions, key=itemgetter(0))

        # s[0] contains the distance-sqaured values
        # s[1] contains the ikSolution

        # to return a sorted list...
        # return [s[1] for s in sortedSolutions]

        # to return the shorted-distance solution...
        return sortedSolutions[0][1]

    def test(self):
        #testing
        xform = self.robot.link(6).getTransform()
        R_wrist = so3.identity()
        t_wrist = xform[1]

        # Setup ik objectives for both arms
        goal = ik.objective(self.robot.link(6),R=R_wrist,t=t_wrist)
        q = self.get_ik_solutions(goal)
        return q
