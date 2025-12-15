import bimanual
import numpy as np
np.set_printoptions(suppress=True)


if __name__ == "__main__":
	# joint 123456
	print(bimanual.forward_kinematics(np.array([0.001,1.1,0.515,0.591,0.0,0.0])))  #joint2pos
	# pos xyzrpy
	print(bimanual.inverse_kinematics(np.array([0.13,0.0003,0.09,0.003 ,-0.005,0.002 ])))  #pos2joint
