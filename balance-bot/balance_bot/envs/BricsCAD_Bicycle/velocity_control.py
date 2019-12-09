import pybullet as p
import time
import pybullet_data
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
planeId = p.loadURDF("plane.urdf")

cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
humanoid = p.loadURDF("BricsCAD_Bicycle.urdf",cubeStartPos, cubeStartOrientation)

############ Add parameters
gravId = p.addUserDebugParameter("gravity", -10, 10, -10)
jointIds = []
paramIds = []
maxForce = 500
jointDict={}
jointDict["Bar"] = dict()
jointDict["Front"] = dict()
jointDict["Back"] = dict()

jointDict["Bar"]["id"] = 0
jointDict["Front"]["id"] = 1
jointDict["Back"]["id"] = 2

jointDict["Back"]["vecParaId"] = p.addUserDebugParameter("BackWheel_veclocity", -500, 500, -100)
jointDict["Front"]["vecParaId"] = p.addUserDebugParameter("FrontWheel_veclocity", -500, 500, 100)
jointDict["Bar"]["positionID"] = p.addUserDebugParameter("Bar_position", -5, 5, 0)


############

p.setPhysicsEngineParameter(numSolverIterations=10)
p.changeDynamics(humanoid, -1, linearDamping=0, angularDamping=0)

for j in range(p.getNumJoints(humanoid)):
  p.changeDynamics(humanoid, j, linearDamping=0, angularDamping=0)

p.setRealTimeSimulation(1)
while (1):
  p.setGravity(0, 0, p.readUserDebugParameter(gravId))
  
  bar_Pos = p.readUserDebugParameter(jointDict["Bar"]["positionID"])
  p.setJointMotorControl2(humanoid, jointDict["Bar"]["id"], p.POSITION_CONTROL, bar_Pos, force=5 * 240.)
  
  back_vec = p.readUserDebugParameter(jointDict["Back"]["vecParaId"])
  p.setJointMotorControl2(humanoid, jointDict["Back"]["id"], controlMode=p.VELOCITY_CONTROL, targetVelocity = back_vec, force = maxForce)

  front_vec = p.readUserDebugParameter(jointDict["Front"]["vecParaId"])
  p.setJointMotorControl2(humanoid, jointDict["Front"]["id"], controlMode=p.VELOCITY_CONTROL, targetVelocity = front_vec, force = maxForce)
  
  cubePos, cubeOrn = p.getBasePositionAndOrientation(humanoid)
  cubeEuler = p.getEulerFromQuaternion(cubeOrn)
  linear, angular = p.getBaseVelocity(humanoid)
  print(cubeEuler)
  

  time.sleep(0.01)
