import numpy as np
import time
import random

from gz.transport13 import Node
from gz.msgs11.entity_factory_pb2 import EntityFactory
from gz.msgs11.pose_pb2 import Pose
from gz.msgs11.boolean_pb2 import Boolean



def spawnModel(node, name, filepath, position, orientation):

    spawn_model_service = "/world/field/create"
    spawn_request = EntityFactory(name = name, sdf_filename = filepath)

    spawn_request.pose.position.x = position[0]
    spawn_request.pose.position.y = position[1]
    spawn_request.pose.position.z = position[2]

    spawn_request.pose.orientation.x = orientation[0]
    spawn_request.pose.orientation.y = orientation[1]
    spawn_request.pose.orientation.z = orientation[2]
    spawn_request.pose.orientation.w = orientation[3]

    return node.request(spawn_model_service, spawn_request, EntityFactory, Boolean, timeout=1000)

def setModelPose(node, name, position, orientation):
    
    set_pose_service = "/world/field/set_pose"
    spawn_request = Pose(name=name)

    spawn_request.position.x = position[0]
    spawn_request.position.y = position[1]
    spawn_request.position.z = position[2]

    spawn_request.orientation.x = orientation[0]
    spawn_request.orientation.y = orientation[1]
    spawn_request.orientation.z = orientation[2]
    spawn_request.orientation.w = orientation[3]

    return node.request(set_pose_service, spawn_request, Pose, Boolean, timeout=1000)




TIME_STEP = 1/30 
ACCELERATION_MAX_STEP = .5
DUMMY_COUNT = 3
MAX_X = 15
MIN_X = -15
MAX_Y = 15
MIN_Y = -15 
# Velocity multiplier when bouncing off of field boundary
DAMPING_FACTOR = 0.5
def main(args=None):
    node = Node()
    dummy_human_sdf = "model://man"
    positions = []
    velocities = []
    accelerations = []
    dummy_names = []

    for num in range(DUMMY_COUNT):
        model_name = "simulation_man_" + str(num)
        # Z postition is 1/2 of model height
        spawn_position = [random.randint(-10,10), random.randint(-10,10), 0.9144] 
        dummy_names.append(model_name)
        positions.append(np.array(spawn_position))
        velocities.append(np.array([0,0,0]))
        accelerations.append(np.array([0,0,0]))
        # Rotation on X axis to stand model on feet and Z to face the camera
        result = spawnModel(node, model_name, dummy_human_sdf, spawn_position, [ 0, 0.7071068, 0.7071068, 0 ])
        if result[0] != True:
            print("Model failed to spawn")

    # Allow time for models to appear in simulation
    time.sleep(5)

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
            setModelPose(node, dummy_names[i], positions[i], [ 0, 0.7071068, 0.7071068, 0 ])

if __name__ == '__main__':
    main()

    
