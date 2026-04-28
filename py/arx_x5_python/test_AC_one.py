from bimanual import SingleArm
import numpy as np
from typing import Dict, Any
import time

def test_dual_arm(left_arm: SingleArm, right_arm: SingleArm):

    while(1):


        # left_arm.gravity_compensation()
        # right_arm.gravity_compensation()



        time.sleep(0.01)



if __name__ == "__main__":
    # Define arm configurations (these should be adjusted based on your specific robot model)
    left_arm_config: Dict[str, Any] = {
        "can_port": "can1",
        "type": 2,
        # 0>2023  2>2025
        # Add necessary configuration parameters for the left arm
    }
    right_arm_config: Dict[str, Any] = {
        "can_port": "can3",
        "type": 2,
        # 0>2023  2>2025
        # Add necessary configuration parameters for the right arm
    }
    # if want change mass.open py/arx_x5_python/bimanual/script/x5_2025.urdf,change link6 mass
    left_arm = SingleArm(left_arm_config)
    right_arm = SingleArm(right_arm_config)
    # Create BimanualArm instance
    # Run the test
    test_dual_arm(left_arm,right_arm)