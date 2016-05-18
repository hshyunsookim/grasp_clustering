from klampt import *
from klampt import vectorops,so3,se3,gldraw,trajectory,visualization,robotplanning
from klampt.cspace import CSpace,MotionPlan
from klampt.robotsim import Geometry3D
from klampt.robotcollide import WorldCollider
from klampt.robotcspace import ClosedLoopRobotCSpace
import math
import os
import random

class LimbCSpace (CSpace):
    """Much of your code for HW4 will go here.  This class
    defines hooks for a motion planner.  Primarily you must define a sampling
    bound and feasibility test.  See klampt/cspace.py for more details
    on what else you can tweak.

    The configuration space is the 7-DOF configuration space of a
    single limb's joint angles.

    Attributes:
        - planner: the LimbPlanner object.  Useful for calling the
          get/set/check methods.
        - limb: the moving limb

    Attributes inherited from CSpace:
        - bounds: configuration sampling bounds, a list of pairs indicating
          the minimum/maximum range on each dimension.
        - eps: edge collision tolerance
    """
    def __init__(self,planner,limb):
        CSpace.__init__(self)
        self.planner = planner
        self.limb = limb
        #TODO: what Cartesian bounding box should the planner sample from?
        self.robot = self.planner.robot
        id_to_index = dict([(self.robot.link(i).getID(),i) for i in range(self.robot.numLinks())])
        if limb=='left':
            # self.limb_indices = left_arm_geometry_indices + left_hand_geometry_indices
            self.limb_indices = self.planner.left_arm_indices
        else:
            # self.limb_indices = right_arm_geometry_indices + right_hand_geometry_indices
            self.limb_indices = self.planner.right_arm_indices
        qmin,qmax = self.robot.getJointLimits()
        self.bound = [(qmin[i]-1e-6,qmax[i]+1e-6) for i in self.limb_indices]
        self.eps = 1e-1

    def feasible(self,q):
        if len(q) != len(self.bound):
            # print len(q), len(self.bound)
            qnew = q[0:len(self.bound)]
            q=qnew
        for i in range(len(q)):
            # print "q", i, q[i], self.bound[i][0], self.bound[i][1]
            if (q[i] < self.bound[i][0]) :
                print "Joint #",self.limb_indices[i],"(",q[i],") out of limits (min:",self.bound[i][0],")"
                print "Changed joint value to its minimum"
                q[i] = self.bound[i][0]

            if (q[i] > self.bound[i][1]) :
                print "Joint #",self.limb_indices[i],"(",q[i],") out of limits (max:",self.bound[i][1],")"
                print "Changed joint value to its maximum"
                q[i] = self.bound[i][1]

        if not CSpace.feasible(self,q):
            # print "LimbCSpace.feasible: Configuration is out of bounds"
            return False

        # cond = self.planner.check_limb_collision_free(self.limb,q)
        # if not cond:
        if not self.planner.check_limb_collision_free(self.limb,q):
            # print "LimbCSpace.feasible: Configuration is in collision"
            return False
        return True


class LimbPlanner:
    """Much of your code for HW4 will go here.

    Attributes:
        - world: the WorldModel
        - robot: the RobotModel
        - collider: a WorldCollider object (see the klampt.robotcollide module)
    """
    def __init__(self,world):
        self.world = world
        self.robot = world.robot(0)
        self.collider = WorldCollider(world)

    def check_collision_free(self):
        armfilter = None
        collindices = set(range(0,self.robot.numLinks()))

        #print "collindices", collindices
        armfilter = lambda x:isinstance(x,RobotModelLink) and (x.index in collindices)

        #check with objects in world model
        for o1,o2 in self.collider.collisionTests(armfilter,lambda x:True):
            #print "Collision Test: Collision between",o1[0].getName(),o2[0].getName()
            #print "Collision Test: Collision for ",o2[0].getName()
            #print o1[0].index, o2[0].index

            if o1[1].collides(o2[1]):
                print "Collision between",o1[0].getName(),o2[0].getName()
                # print "Collision between",o1[0].index,o2[0].index
                return False
        return True

    def plan_limb(self,limb,limbstart,limbgoal, printer=True, iks = None):
        """Returns a 7-DOF milestone path for the given limb to move from the
        start to the goal, or False if planning failed"""
        self.rebuild_dynamic_objects()

        # NOTE: Not sure, but I think this is what's happening
        # Need to reset pathToDraw here because the MyGLViewer.draw() is called
        # concurrently, it uses an old path (ex. of left arm) while using the
        # new limb (ex. right limb) for drawing the trajectory. This throws an
        # Index-Out-of-Range exception for drawing the path
        self.pathToDraw = []

        # NOTE:
        cspace = LimbCSpace(self,limb)
        if iks != None:
            print "Initializing ClosedLoopCSPace"
            cspace = ClosedLoopCSpaceTest(self,limb,iks)

        if not cspace.feasible(limbstart):
            print "  Start configuration is infeasible!"
            return 1
        if not cspace.feasible(limbgoal):
            print "  Goal configuration is infeasible!"
            return 2

        MotionPlan.setOptions(type="rrt", perturbationRadius = 1, connectionThreshold=2, bidirectional = True, shortcut = True, restart=True)
        plan = MotionPlan(cspace)

        plan.setEndpoints(limbstart,limbgoal)
        # maxPlanIters = 20
        # maxSmoothIters = 100
        maxPlanIters = 10
        maxSmoothIters = 100
        print "  Planning.",
        for iters in xrange(maxPlanIters):
            # print iters

            if limb=='left':
                self.limb_indices = self.left_arm_indices
                self.activeLimb = 'left'
            else:
                self.limb_indices = self.right_arm_indices
                self.activeLimb = 'right'

            self.roadmap = plan.getRoadmap()
            V,E =self.roadmap
            print ".",

            plan.planMore(10)                                       # 100

            path = plan.getPath()
            if path != None:
                if printer:
                    print "\n  Found a path on iteration",iters
                if len(path) > 2:
                    print "  Smoothing path"
                    # plan.planMore(min(maxPlanIters-iters,maxSmoothIters))
                    plan.planMore(maxSmoothIters)
                    path = plan.getPath()
                cspace.close()
                plan.close()

                self.pathToDraw = path

                return path
        cspace.close()
        plan.close()
        if printer:
            print "  No path found"

        return False

    def plan(self,start,goal,limb,printer=True, iks = None, ignoreColShelfSpatula = True):
        """Plans a motion for the robot to move from configuration start
        to configuration goal.  By default, moves the left arm first,
        then the right.  To move the right first, set the 'order' argument
        to ['right','left']"""
        limbstart = {}
        limbgoal = {}
        # for l in ['left','right']:
        l =limb
        limbstart[l] = self.get_limb_config(start,l)
        limbgoal[l] = self.get_limb_config(goal,l)
        path = [start]
        curconfig = start[:]

        diff = sum((a-b)**2 for a,b in zip(limbstart[l],limbgoal[l]))
        if diff > 1e-8:
            if printer:
                print "< Planning for limb",l,">"
                # print "  Euclidean distance:",math.sqrt(diff)
            self.robot.setConfig(curconfig)

            #do the limb planning
            if not ignoreColShelfSpatula:
                self.left_arm_indices = left_arm_geometry_indices + left_hand_geometry_indices + [55,56,57]
            else:
                self.left_arm_indices = left_arm_geometry_indices + left_hand_geometry_indices
            # print "  Left arm links", self.left_arm_indices

            if iks == None:
                limbpath = self.plan_limb(l,limbstart[l],limbgoal[l],printer=printer, iks=iks)

            else:
                trajectory = self.plan_closedLoop(start,goal,iks=iks)
                return trajectory

            if limbpath == 1 or limbpath == 2 or limbpath == False:
                if printer:
                    print "  Failed to plan for limb",l,"\n"
                return limbpath
            if printer:
                print "  Planned successfully for limb",l, "\n"
            #concatenate whole body path
            for qlimb in limbpath[1:]:
                q = path[-1][:]
                self.set_limb_config(l,qlimb,q)
                path.append(q)
            self.set_limb_config(l,limbgoal[l],curconfig)
        return path

    def plan_closedLoop(self,qstart,qgoal,iks,printer=False):
        if printer:
            print "starting config: ", qstart
            print "goal config: ", qgoal
        self.world.terrain(0).geometry().setCollisionMargin(0.05)

        cspace = robotcspace.ClosedLoopRobotCSpace(self.robot, iks, self.collider)
        cspace.eps = 1e-3
        cspace.setup()

        configs=[]
        configs.append(qstart)
        configs.append(qgoal)

        configs[0] = cspace.solveConstraints(configs[0])
        configs[1] = cspace.solveConstraints(configs[1])

        settings = { 'type':"sbl", 'perturbationRadius':0.5, 'bidirectional':1, 'shortcut':1, 'restart':1, 'restartTermCond':"{foundSolution:1,maxIters:1000}" }

        wholepath = [configs[0]]
        for i in range(len(configs)-1):
            #this code uses the robotplanning module's convenience functions
            self.robot.setConfig(configs[i])
            plan = robotplanning.planToConfig(self.world,self.robot,configs[i+1],
                                              movingSubset='all',
                                              **settings)

            if plan is None:
                return False
            print "  Planning..."

            keepPlanning = True
            while keepPlanning:
                plan.planMore(500)

                #this code just gives some debugging information. it may get expensive
                # V,E = plan.getRoadmap()
                # self.roadmap = plan.getRoadmap()
                # print "  ", len(V),"feasible milestones sampled,",len(E),"edges connected"
                path = plan.getPath()
                if path is None or len(path)==0:
                    print "Failed to plan path"
                    #debug some sampled configurations
                    # print V[0:max(10,len(V))]
                    # return False
                else:
                    keepPlanning = False

            self.pathToDraw = path

            #the path is currently a set of milestones: discretize it so that it stays near the contact surface
            path = cspace.discretizePath(path,epsilon=1e-2)
            # path = cspace.discretizePath(path,epsilon=1e-4)
            wholepath += path[1:]

            #to be nice to the C++ module, do this to free up memory
            plan.space.close()
            plan.close()

        return wholepath