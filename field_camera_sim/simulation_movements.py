import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import random

import rclpy
from rclpy.node import Node as RosNode
from gz.transport13 import Node
from gz.msgs11.entity_factory_pb2 import EntityFactory
from gz.msgs11.pose_pb2 import Pose
from gz.msgs11.boolean_pb2 import Boolean



class GzMessage:
    def __init__(self):
        self.node = Node()

    def spawnModel(self, name, filepath, position, orientation):

        spawn_model_service = "/world/field/create"
        spawn_request = EntityFactory(name = name, sdf_filename = filepath)

        spawn_request.pose.position.x = position[0]
        spawn_request.pose.position.y = position[1]
        spawn_request.pose.position.z = position[2]

        spawn_request.pose.orientation.x = orientation[0]
        spawn_request.pose.orientation.y = orientation[1]
        spawn_request.pose.orientation.z = orientation[2]
        spawn_request.pose.orientation.w = orientation[3]

        return self.node.request(spawn_model_service, spawn_request, EntityFactory, Boolean, timeout=1000)

    def setModelPose(self, name, position, orientation):
        
        set_pose_service = "/world/field/set_pose"
        spawn_request = Pose(name=name)

        spawn_request.position.x = position[0]
        spawn_request.position.y = position[1]
        spawn_request.position.z = position[2]

        spawn_request.orientation.x = orientation[0]
        spawn_request.orientation.y = orientation[1]
        spawn_request.orientation.z = orientation[2]
        spawn_request.orientation.w = orientation[3]

        return self.node.request(set_pose_service, spawn_request, Pose, Boolean, timeout=1000)
    
def getParam(name, default, type):
    param = RosNode("Parameter_Collector")
    param.declare_parameter(name, default)
    if type == "int":
        result = param.get_parameter(name).get_parameter_value().integer_value
    elif type == "str":
        result = param.get_parameter(name).get_parameter_value().string_value
    elif type == "bool":
        result = param.get_parameter(name).get_parameter_value().bool_value
    param.destroy_node()
    return result
    

def main(args=None):

    TIME_STEP = 1/60 
    ACCELERATION_MAX_STEP = .5
    MAX_X = 15
    MIN_X = -15
    MAX_Y = 15
    MIN_Y = -15 
    # Velocity multiplier when bouncing off of field boundary
    DAMPING_FACTOR = 0.5
    gz = GzMessage()
    rclpy.init()
    DUMMY_COUNT = getParam("num_dummies", 3, "int")

    apriltag_model = "model://AprilTag"
    dummy_human_model = "model://man"

    positions = []
    velocities = []
    accelerations = []
    dummy_names = []

    time.sleep(5)

    # Spawn AprilTag
    gz.spawnModel("simulation_AprilTag", apriltag_model, [0, 15, 0], [0, 0, 0, 0])
    # Spawn Dummies
    for num in range(DUMMY_COUNT):
        model_name = "simulation_man_" + str(num)
        # Z postition is 1/2 of model height
        spawn_position = [random.randint(-10,10), random.randint(-10,10), 0.9144] 
        dummy_names.append(model_name)
        positions.append(np.array(spawn_position))
        velocities.append(np.array([0,0,0]))
        accelerations.append(np.array([0,0,0]))
        # Rotation on X axis to stand model on feet and Z to face the camera
        result = gz.spawnModel(model_name, dummy_human_model, spawn_position, [ 0, 0.7071068, 0.7071068, 0 ])
        if result[0] != True:
            print("Model failed to spawn")

    # Allow time for models to appear in simulation
    time.sleep(2)

    while True:
        time.sleep(TIME_STEP)
        for i in range(DUMMY_COUNT):
            
            if positions[i][0] > MAX_X:
                positions[i][0] = MAX_X
                velocities[i][0] *= -DAMPING_FACTOR
                accelerations[i][0] = 0
            elif positions[i][0] < MIN_X:
                positions[i][0] = MIN_X
                velocities[i][0] *= -DAMPING_FACTOR
                accelerations[i][0] = 0

            if positions[i][1] > MAX_Y:
                positions[i][1] = MAX_Y
                velocities[i][1] *= -DAMPING_FACTOR
                accelerations[i][1] = 0
            elif positions[i][1] < MIN_Y:
                positions[i][1] = MIN_Y
                velocities[i][1] *= -DAMPING_FACTOR
                accelerations[i][1] = 0

            positions[i] = positions[i] + TIME_STEP * velocities[i] 
            velocities[i] = velocities[i] + TIME_STEP * accelerations[i]
            change_acc_x = random.uniform(-ACCELERATION_MAX_STEP, ACCELERATION_MAX_STEP)
            change_acc_y = random.uniform(-ACCELERATION_MAX_STEP, ACCELERATION_MAX_STEP)
            accelerations_change = np.array([change_acc_x, change_acc_y, 0])
            accelerations[i] = accelerations[i] + (TIME_STEP * accelerations_change)
            gz.setModelPose(dummy_names[i], positions[i], [ 0, 0.7071068, 0.7071068, 0 ])

if __name__ == '__main__':
    main()

    
