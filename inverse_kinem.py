def sysCall_init():
 sim = require('sim')
 simROS2 = require('simROS2')
 simIK = require('simIK')
    
 self.publisher = simROS2.createPublisher('/tipPosition', 'geometry_msgs/msg/Vector3')
    
 self.joints = []
    
 for i in range(7):
     self.joints.append(sim.getObject(f'/YuMi/leftJoint{i + 1}'))
    
 self.ikJoints = []

 self.simBase = sim.getObject('/YuMi')
 self.simTip = sim.getObject('../attachPoint/Stick/Tip')
 self.simTarget = sim.getObject('/Target')
 self.ikEnv = simIK.createEnvironment()

 self.ikBase = simIK.createDummy(self.ikEnv)  # create a dummy in the IK environment
 # set that dummy into the same pose as its CoppeliaSim counterpart:
 simIK.setObjectMatrix(self.ikEnv, self.ikBase, -1, sim.getObjectMatrix(self.simBase))
 parent = self.ikBase

 # loop through all joints
 for i in range(len(self.joints)):
     # create a joint in the IK environment:
     self.ikJoints.append(simIK.createJoint(self.ikEnv, simIK.jointtype_revolute))
     # set it into IK mode:
     simIK.setJointMode(self.ikEnv, self.ikJoints[i], simIK.jointmode_ik)
     # set the same joint limits as its CoppeliaSim counterpart joint:
     cyclic, interv = sim.getJointInterval(self.joints[i])
     simIK.setJointInterval(self.ikEnv, self.ikJoints[i], cyclic, interv)
     # set the same lin./ang. joint position as its CoppeliaSim counterpart joint:
     simIK.setJointPosition(self.ikEnv, self.ikJoints[i], sim.getJointPosition(self.joints[i]))
     # set the same object pose as its CoppeliaSim counterpart joint:
     simIK.setObjectMatrix(self.ikEnv, self.ikJoints[i], -1, sim.getObjectMatrix(self.joints[i]))
     simIK.setObjectParent(self.ikEnv, self.ikJoints[i], parent)  # set its corresponding parent
     parent = self.ikJoints[i]

 self.ikTip = simIK.createDummy(self.ikEnv)  # create the tip dummy in the IK environment
 # set that dummy into the same pose as its CoppeliaSim counterpart:
 simIK.setObjectMatrix(self.ikEnv, self.ikTip, -1, sim.getObjectMatrix(self.simTip))
 simIK.setObjectParent(self.ikEnv, self.ikTip, parent)  # attach it to the kinematic chain
 self.ikTarget = simIK.createDummy(self.ikEnv)  # create the target dummy in the IK environment
 # set that dummy into the same pose as its CoppeliaSim counterpart:
 simIK.setObjectMatrix(self.ikEnv, self.ikTarget, -1, sim.getObjectMatrix(self.simTarget))
 simIK.setTargetDummy(self.ikEnv, self.ikTip, self.ikTarget)  # link the two dummies
 self.ikGroup = simIK.createGroup(self.ikEnv)  # create an IK group
 # set its resolution method to undamped:
 simIK.setGroupCalculation(self.ikEnv, self.ikGroup, simIK.method_pseudo_inverse, 0, 3)
 # add an IK element to that IK group:
 ikElement = simIK.addElement(self.ikEnv, self.ikGroup, self.ikTip)
 # specify the base of that IK element:
 simIK.setElementBase(self.ikEnv, self.ikGroup, ikElement, self.ikBase)
 # specify the constraints of that IK element:
 simIK.setElementConstraints(self.ikEnv, self.ikGroup, ikElement, simIK.constraint_x | simIK.constraint_y | simIK.constraint_z)
 

def sysCall_actuation():  
 data = {key: val for key, val in zip(['x', 'y', 'z'], sim.getObjectPosition(self.simTip))}
 simROS2.publish(self.publisher, data)
    
 # reflect the pose of the target dummy in the IK environment:
 simIK.setObjectMatrix(self.ikEnv, self.ikTarget, self.ikBase, sim.getObjectMatrix(self.simTarget, self.simBase))
 simIK.handleGroup(self.ikEnv, self.ikGroup)  # solve
 # apply the calculated joint values:
 for i in range(len(self.joints)):
     sim.setJointTargetPosition(self.joints[i], simIK.getJointPosition(self.ikEnv, self.ikJoints[i]))


def sysCall_cleanup():
 # Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
 simROS2.shutdownPublisher(self.publisher)
