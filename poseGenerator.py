import math, random
from klampt import vectorops, se3, so3, loader, gldraw, ik, contact,robotcollide
import planner
from operator import itemgetter

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
        self.pose_range = [math.pi/180*2,math.pi/180*2,math.pi/180*2]

    def randomPose(self, range = 1):
        qmin,qmax = self.robot.getJointLimits()
        center = self.controller.getCommandedConfig()
        q = center
        rangeVal = self.base_range+self.pose_range

        for j in base_links:
            q[j] = random.uniform(max(qmin[j],center[j]-rangeVal[j]*range),min(qmax[j],center[j]+rangeVal[j]*range))

        return q

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