from bimanual import BimanualArm
import numpy as np
from typing import Dict, Any

def test_dual_arm(dual_arm: BimanualArm, duration: float = 10.0, dt: float = 0.01):
    #single_arm.go_home()
    while(1):
        # Example positions and orientations for the arms
        left_position = np.array([0.0, 0.0, 0.1])  # Example position (x, y, z)
        left_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # Example orientation as a quaternion

        right_position = np.array([0.0, 0.0, 0.1])  # Example position (x, y, z)
        right_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # Example orientation as a quaternion
        
        poses = {
            'left': (left_position, left_orientation),
            'right': (right_position, right_orientation)
        }
        dual_arm.set_ee_pose(poses)
        
        #position = [0.0, 0.1, 0.1]  # x, y, z 位置
        #quaternion = [1.0, 0.0, 0.0, 0.0]  # 四元数表示方向

        #success = dual_arm.set_ee_pose(pos=position, quat=quaternion)
        
        #print(single_arm.get_ee_pose())
        #print(single_arm.get_joint_positions())
        
        #positions = [0.5, 1.0, -0.5]  # 指定每个关节的位置
        #joint_names = ["joint1", "joint2", "joint3"]  # 对应关节的名称

        #success = single_arm.set_joint_positions(positions=positions, joint_names=joint_names)

if __name__ == "__main__":
    # Define arm configurations (these should be adjusted based on your specific robot model)
    left_arm_config: Dict[str, Any] = {
        "can_port": "can1",
        # Add necessary configuration parameters for the left arm
    }
    right_arm_config: Dict[str, Any] = {
        "can_port": "can0",
        # Add necessary configuration parameters for the right arm
    }

    # Create BimanualArm instance
    bimanual_arm = BimanualArm(left_arm_config, right_arm_config)

    # Run the test
    test_dual_arm(bimanual_arm)