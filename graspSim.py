#!/usr/bin/python

from klampt import robotsim
from klampt.glprogram import *
from klampt import vectorops, se3, so3, loader, gldraw, ik, contact,robotcollide
from klampt.robotsim import Geometry3D
from klampt import visualization
# from baxter import *
import os
import math
import random
import copy
from threading import Thread,Lock
from Queue import Queue
from operator import itemgetter

# The path of the klampt_models directory
model_dir = "klampt_models/"

# Joint numbers
joints = [2, 3, 16, 17, 30]

# simulation mode
global FAKE_SIMULATION
FAKE_SIMULATION = 0

dq = 1.0

def draw_xformed(xform,localDrawFunc):
    """Draws something given a se3 transformation and a drawing function
    that draws the object in its local frame.

    E.g., draw_xformed(xform,lambda:gldraw.box([ax,ay,az],[bx,by,bz])) draws
    a box oriented and translated by xform."""
    mat = zip(*se3.homogeneous(xform))
    mat = sum([list(coli) for coli in mat],[])

    glPushMatrix()
    glMultMatrixf(mat)
    localDrawFunc()
    glPopMatrix()
def draw_oriented_box(xform,bmin,bmax):
    """Helper: draws an oriented box"""
    draw_xformed(xform,lambda:gldraw.box(bmin,bmax))
def draw_wire_box(bmin,bmax):
    """Helper: draws a wireframe box"""
    glBegin(GL_LINE_LOOP)
    glVertex3f(bmin[0],bmin[1],bmin[2])
    glVertex3f(bmin[0],bmin[1],bmax[2])
    glVertex3f(bmin[0],bmax[1],bmax[2])
    glVertex3f(bmin[0],bmax[1],bmin[2])
    glEnd()
    glBegin(GL_LINE_LOOP)
    glVertex3f(bmax[0],bmin[1],bmin[2])
    glVertex3f(bmax[0],bmin[1],bmax[2])
    glVertex3f(bmax[0],bmax[1],bmax[2])
    glVertex3f(bmax[0],bmax[1],bmin[2])
    glEnd()
    glBegin(GL_LINES)
    glVertex3f(bmin[0],bmin[1],bmin[2])
    glVertex3f(bmax[0],bmin[1],bmin[2])
    glVertex3f(bmin[0],bmin[1],bmax[2])
    glVertex3f(bmax[0],bmin[1],bmax[2])
    glVertex3f(bmin[0],bmax[1],bmax[2])
    glVertex3f(bmax[0],bmax[1],bmax[2])
    glVertex3f(bmin[0],bmax[1],bmin[2])
    glVertex3f(bmax[0],bmax[1],bmin[2])
    glEnd()
def draw_oriented_wire_box(xform,bmin,bmax):
    """Helper: draws an oriented wireframe box"""
    draw_xformed(xform,lambda:draw_wire_box(bmin,bmax))
def draw_vector(x,n,k):
    glDisable(GL_LIGHTING)
    glColor3f(1,1,0)
    glLineWidth(5.0)
    glBegin(GL_LINES)
    glVertex3f(x[0],x[1],x[2])
    glVertex3f(x[0]+k*n[0],x[1]+k*n[1],x[2]+k*n[2])
    glEnd()
    glEnable(GL_LIGHTING)



class LowLevelController:
    """A low-level interface to the Baxter robot (with parallel jaw
    grippers).  Does appropriate locking for multi-threaded use.
    You should use this in your picking controller."""
    def __init__(self,robotModel,robotController):
        self.robotModel = robotModel
        self.controller = robotController
        self.lock = Lock()
    def getSensedConfig(self):
        self.lock.acquire()
        res = self.controller.getSensedConfig()
        self.lock.release()
        return res
    def getSensedVelocity(self):
        self.lock.acquire()
        res = self.controller.getSensedVelocity()
        self.lock.release()
        return res
    def getCommandedConfig(self):
        self.lock.acquire()
        res = self.controller.getCommandedConfig()
        self.lock.release()
        return res
    def getCommandedVelocity(self):
        self.lock.acquire()
        res = self.controller.getCommandedVelocity()
        self.lock.release()
        return res
    def setPIDCommand(self,configuration,velocity):
        """Sets the controller to a PID command mode"""
        self.lock.acquire()
        self.controller.setPIDCommand(configuration,velocity)
        self.lock.release()
    def setMilestone(self,destination,endvelocity=None):
        """Immediately sets the motion queue to move to the given
        milestone.  If endvelocity is given, then the end of the
        queue will be moving at that velocity.  Otherwise, the end
        velocity will be zero."""
        self.lock.acquire()
        if endvelocity == None: self.controller.setMilestone(destination,0)
        else: self.controller.setMilestone(destination,endvelocity)
        self.lock.release()
    def appendMilestone(self,destination,endvelocity=None):
        """Appends a milestone to the motion queue.  If endvelocity
        is given, then the end of the queue will be moving at that velocity.
        Otherwise, the end velocity will be zero."""
        self.lock.acquire()
        if endvelocity == None: self.controller.addMilestone(destination)
        else: self.controller.addMilestone(destination,endvelocity)
        self.lock.release()
    def isMoving(self):
        return self.controller.remainingTime()>0
    def remainingTime(self):
        return self.controller.remainingTime()
    def commandGripper(self,command):
        """Sends the command to the indicated gripper.
        For the parallel-jaw gripper, [0] is closed, [1] is open
        Warning: don't do this while moving"""
        self.lock.acquire()
        q = self.controller.getCommandedConfig()
        self.robotModel.setConfig(q)
        completed = set_model_gripper_command(self.robotModel,command)
        self.controller.setMilestone(self.robotModel.getConfig())
        self.lock.release()
        return completed
    def moveHand(self,command):
        """Sends the command to the indicated gripper.
        For the parallel-jaw gripper, [0] is closed, [1] is open
        Warning: don't do this while moving"""
        self.lock.acquire()
        q = self.controller.getCommandedConfig()
        self.robotModel.setConfig(q)
        set_gripper_location_command(self.robotModel,command)
        self.controller.setMilestone(self.robotModel.getConfig())
        self.lock.release()
    def randomMoveHand(self):
        self.lock.acquire()
        q = self.controller.getCommandedConfig()

        # height = 0.5
        q[1] = 0.5
        self.robotModel.setConfig(q)
        self.controller.setMilestone(q)

        # random pose
        self.controller.addMilestone(generate_random_pose(self.robotModel))

        self.lock.release()


def set_model_gripper_command(robot,command):
    value = command[0]
    global dq

    # set dq when opening fingers
    if value == -1 :
        dq = 0.01

    # grasp is completed
    if dq<0.01:
        print "grasp completed, object in hand"
        return True



    q = robot.getConfig()
    oldConfig = robot.getConfig()
    for linkNum in [9,14,18]:
        q[linkNum] = q[linkNum] + value*dq

    # if joint goes out of limits before getting into contact with object
    qmin, qmax = robot.getJointLimits()
    for i in range(robot.numLinks()):
        if qmin[i] > q[i] or qmax[i] < q[i]:
            print "grasp failed"
            return True


    robot.setConfig(q)
    for i in range(robot.numLinks()):
        if robot.link(i).geometry().collides(simWorld.rigidObject(0).geometry()):
            dq = dq/2
            # print "reverting to previous time-stamp, halving dq to", round(dq,4)
            robot.setConfig(oldConfig)

    # grasp is not completed
    return False

def set_gripper_location_command(robot,command):
    linkNum = int(command[0]) - 1

    # Drivers: [6(swivel),7,9(swivel),10,12]
    # Links: [0,1,2,3,4,5]
    qmin,qmax = robot.getJointLimits()

    q = robot.getConfig()
    # q[linkNum] = random.uniform(qmin[linkNum],qmax[linkNum])
    if linkNum <3:
        q[linkNum] = q[linkNum] + 0.01
    else:
        q[linkNum] = q[linkNum] + 0.1
    robot.setConfig(q)

    # robot.driver(driverNum).setValue(robot.driver(driverNum).getValue() + 0.1)
    # robot.driver(1).setValue(robot.driver(10).getValue() + value*0.05)
    # robot.driver(2).setValue(robot.driver(12).getValue() + value*0.05)

def generate_random_pose(robot):
    q = robot.getConfig()
    links = [3,4,5]
    for link in links:
        q[link] = random.uniform(-math.pi, math.pi)
    return q

def generate_trajectory(qi, qf):
    i = 0
    endIndex = 2
    path = [0.0, 0.0]
    path[0] = qi
    path[1] = qf
    print qi, qf
    while i < endIndex-1:
        # print i, endIndex
        q = path[i]
        qNext = path[i+1]
        dt = vectorops.distance(q,qNext)

        # smooth trajectory by interpolating between two consecutive configurations
        # if the distance between the two is big
        if dt>0.1:
            qInterp = vectorops.div(vectorops.add(q, qNext), 2)
            path.insert(i+1, qInterp)
            endIndex +=1
            continue
        else:
            i += 1
    print len(path)
    return path

# this function is called on a thread
def run_controller(controller,command_queue):
    # simply reveals the shelf xform
    while True:
        c = command_queue.get()
        if c != None:
            # print "Running command",c
            if c >= 'a' and c <= 'l':
                controller.viewBinAction('bin_'+c.upper())
            elif c == 'x':
                completed = False
                global dq
                dq = 1.0
                while not completed:
                    completed = controller.commandGripper([1])
                    time.sleep(0.01)
            elif c == 'u':
                controller.commandGripper([-1])
            elif c == 'r':
                controller.randomMoveHand()
            else:
                controller.moveHand(c)
        else:
            print "Waiting for command..."
            time.sleep(0.1)
    print "Done"

class MyGLViewer(GLRealtimeProgram):
    def __init__(self,simworld,planworld):
        GLRealtimeProgram.__init__(self,"My GL program")
        self.simworld = simworld
        self.planworld = planworld
        self.sim = robotsim.Simulator(simworld)
        self.simulate = True
        # self.sim.simulate(0)

        if FAKE_SIMULATION:
            self.low_level_controller = FakeLowLevelController(simworld.robot(0), self.sim.controller(0))
        else:
            self.low_level_controller = LowLevelController(simworld.robot(0), self.sim.controller(0))

        self.command_queue = Queue()
        self.thread = Thread(target=run_controller,args=(self.low_level_controller,self.command_queue))
        self.thread.start()

        # enable constact feedback
        self.sim.enableContactFeedbackAll()

        # visual settings
        self.showFrames = False

    def idle(self):
        if self.simulate:
            self.sim.simulate(self.dt)
            glutPostRedisplay()

    def drawContactForces(self):
        contacted=False;
        for i in range(self.simworld.numIDs()):
            for j in range(i+1, self.simworld.numIDs()):
                if(self.sim.hadContact(i,j)):
                    if(not contacted):
                        # print "Touching bodies:\n"
                        contacted=True
                    f = self.sim.meanContactForce(i,j)
                    contacts = self.sim.getContacts(i,j)

                    # print self.simworld.getName(i),self.simworld.getName(j), len(contacts)
                    x=[0,0,0]
                    n=[0,0,0]
                    k=0
                    scale = 3

                    if len(contacts) != 0:
                        for numContact in range(len(contacts)):
                            x = vectorops.add(x,contacts[numContact][0:3])
                            n = vectorops.add(n,contacts[numContact][3:6])
                            k += contacts[numContact][6]
                        x = vectorops.div(x,len(contacts))
                        n = vectorops.div(n,len(contacts))
                        k = k/(len(contacts)*scale)
                        draw_vector(x,n,k)

    def display(self):
        #draw the world
        self.sim.updateWorld()
        self.simworld.drawGL()

        #draw commanded configurations
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])
        # only 1 robot in this case, but still use for-loop for generality
        for i in xrange(self.simworld.numRobots()):
            r = self.simworld.robot(i)
            q = self.low_level_controller.getCommandedConfig()
            r.setConfig(q)
            r.drawGL(False)
        glDisable(GL_BLEND)

        self.drawContactForces()

                    # print self.sim.inContact(i,j)
                    # print self.simworld.getName(i),"-",self.simworld.getName(j), f
              # t = self.sim.mean
              # printf("%s - %s: force %g %g %g\n",world.GetName(i).c_str(),world.GetName(j)).c_str(),f.x,f.y,f.z,t.x,t.y,t.z);


        # print contact.simContactMap(self.sim)
        # qSim = self.simworld.robot(0).getConfig()
        # qReal = self.low_level_controller.getSensedConfig()
        # diff = vectorops.distance(qSim, qReal)
        # print diff
        # print self.sim.getActualTorques(0)[1]
        # print self.sim.getContactForces(1,2)
        # print self.simworld.robot(0).getID()
        # print self.simworld.rigidObject(0).getID()
        # self.sim.enableContactFeedbackAll()
        # print max(abs(self.sim.getJointForces(self.simworld.robot(0).link(9))[3:6]))
        # print max(abs(self.sim.getJointForces(self.simworld.robot(0).link(14))[3:6]))
        # print max(abs(self.sim.getJointForces(self.simworld.robot(0).link(18))[3:6]))

        # print self.sim.getJointForces(self.simworld.robot(0).link(9))


        # Show world frame and shelf frame
        if self.showFrames:
            gldraw.xform_widget(se3.identity(), 0.15, 0.017, lighting=True, fancy=True)
            gldraw.xform_widget(self.simworld.robot(0).link(0).getParentTransform(), 0.15, 0.017, lighting=True, fancy=True)
        return

    def keyboardfunc(self,c,x,y):
        c = c.lower()
        if c=='z':
            self.simulate = not self.simulate
            print "Simulating:",self.simulate
        else:
            self.command_queue.put(c)
            if c=='q':
                self.picking_thread.join()
                exit(0)
        glutPostRedisplay()

def load_world():
    world = robotsim.WorldModel()
    print "Loading reflex hands"
    # world.loadElement(os.path.join(model_dir,"reflex.rob"))
    world.loadElement(os.path.join(model_dir,"reflex_col_with_moving_base.rob"))
    print "Loading plane model..."
    world.loadElement(os.path.join(model_dir,"plane.env"))

    R,t = world.robot(0).link(0).getParentTransform()
    R_reorient = so3.rotation([1,0,0], math.pi/2)
    offset = (0,0.15,0.15)

    R = so3.mul(R,R_reorient)
    t = vectorops.add(t, offset)
    world.robot(0).link(0).setParentTransform(R,t)

    for i in range(world.robot(0).numLinks()):
        world.robot(0).link(i).geometry().setCollisionMargin(0)

    return world

def load_item_geometry(bmin,bmax,geometry_ptr = None):
    """Loads the geometry of the given item and returns it.  If geometry_ptr
    is provided, then it is assumed to be a Geometry3D object and the object
    geometry is loaded into it."""
    if geometry_ptr == None:
        geometry_ptr = Geometry3D()

    fn = model_dir + "cube.tri"
    # fn = model_dir + "items/oreo_mega_stuf/textured_meshes/optimized_poisson_textured_mesh.ply"
    if not geometry_ptr.loadFile(fn):
        print "Error loading cube file",fn
        exit(1)
    center = vectorops.mul(vectorops.add(bmin,bmax),0.5)
    scale = [bmax[0]-bmin[0],0,0,0,bmax[1]-bmin[1],0,0,0,bmax[2]-bmin[2]]
    translate = vectorops.sub(bmin,center)

    geometry_ptr.transform(scale,translate)
    geometry_ptr.setCurrentTransform(so3.identity(),[0,0,0])
    return geometry_ptr

def spawn_objects(world):
    """For all ground_truth_items, spawns RigidObjects in the world
    according to their sizes / mass properties"""

    print "Initializing world objects"

    obj = world.makeRigidObject('object1')
    bmin = [0,0,0]
    bmax = [0.05,0.05,0.25]

    mass = 0.01
    m = obj.getMass()
    m.setMass(mass)
    m.setCom([0,0,0])
    m.setInertia(vectorops.mul([bmax[0]-bmin[0],bmax[1]-bmin[1],bmax[2]-bmin[2]],mass/12.0))
    obj.setMass(m)

    c = obj.getContactParameters()
    c.kFriction = 0.6
    c.kRestitution = 0.1;
    c.kStiffness = 10
    c.kDamping = 10
    obj.setContactParameters(c)

    simgeometry = obj.geometry()
    load_item_geometry(bmin,bmax,simgeometry)

    obj.setTransform(so3.identity(),[0.0,0,0.15])
    return

def myCameraSettings(visualizer):
    visualizer.camera.tgt = [0, 0, -0.25]
    visualizer.camera.rot = [0,0.5,0.9]
    visualizer.camera.dist = 1
    visualizer.fov = 60
    visualizer.width = 960
    visualizer.height = 720
    return

if __name__ == "__main__":
    simWorld = load_world()
    world = load_world()

    # spawn_objects(world)
    spawn_objects(simWorld)

    # collider
    col = robotcollide.WorldCollider(simWorld)

    #run the visualizer
    visualizer = MyGLViewer(simWorld,world)
    myCameraSettings(visualizer)
    visualizer.run()
