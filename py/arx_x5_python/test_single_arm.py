from bimanual import SingleArm
from typing import Dict, Any
import numpy as np
import time

def test_single_arm(single_arm: SingleArm, duration: float = 10.0, dt: float = 0.01):

    while(1):


        # single_arm.gravity_compensation()


        # xyzrpy = np.array([0.0, 0.0, 0.01,0.0, 0.0, 0.0])  # x, y, z 位置
        # single_arm.set_ee_pose_xyzrpy(xyzrpy)
        # single_arm.set_gripper_pos(-1)



        # positions = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # 指定每个关节的位置
        # joint_names = ["joint1", "joint2", "joint3","joint4", "joint5", "joint6"]  # 对应关节的名称
        # single_arm.set_joint_positions(positions=positions, joint_names=joint_names)        
        # single_arm.set_gripper_pos(-1)

        
        # print(single_arm.get_ee_pose_xyzrpy())
        # print(single_arm.get_joint_positions())
        # print(single_arm.get_joint_velocities())
        # print(single_arm.get_joint_currents())


        time.sleep(0.01)



if __name__ == "__main__":
    arm_config: Dict[str, Any] = {
        "can_port": "can1",
        "type": 2,  
        # 0>2023  2>2025
    }
    single_arm = SingleArm(arm_config)
    test_single_arm(single_arm)