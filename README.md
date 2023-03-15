# 6-DOF Robot Arm Kinematics

This repository defines a class that solves the forward kinematics (FK) and inverse kinematics (IK) of a 6 degree of freedom (DOF) robot arm.


## Forward Kinematics `SolveFK()`

This code implements the forward kinematics solution for a 6 degree-of-freedom robotic arm. The function takes in a 6D joint input and returns the corresponding 6D pose output. The input joint angles are first converted from degrees to radians and stored in an array `q_in`. The `q` array is then calculated by adding the DH parameter `DH_matrix[i][0]` to each `q_in[i]` value.

The rotation matrices for each joint are then calculated using the R array. The rotation matrices are used to determine the position and orientation of the end effector with respect to the base of the robotic arm. These matrices are multiplied together to get the final transformation matrix `R06`.

The `L` arrays contain the distances between the joint axes and the position of the end effector. The matrices are multiplied by the rotation matrices to obtain the position of the end effector relative to the base.

Finally, the 6D pose output is calculated by combining the position and orientation of the end effector. The Euler angles are extracted from `R06` using the `RotMatToEulerAngle` function. The 6D pose output is returned as a structure containing the X, Y, and Z positions, and the A, B, and C orientation angles, as well as the rotation matrix `R06`. The function returns a boolean value indicating whether the forward kinematics solution was successfully calculated.


# Inverse Kinematics `SolveIK()`

This code is implementing an inverse kinematics (IK) solver for a 6-DOF robotic arm. The IK solver calculates the joint angles required for the arm to achieve a desired end-effector pose, given as a 6D pose (position and orientation) in space.

The code defines several variables and matrices to be used in the calculations. Then, the input pose `_inputPose6D` is extracted and converted into a matrix format. The last joint angles `_lastJoint6D` are also extracted from the input for use in the IK calculation. It calculates the possible joint angles that would achieve the desired end-effector pose in the `_outputSolves` variable.

The code then proceeds to calculate the first joint angle (shoulder elevation angle) based on the position of the end-effector. The IK solver assumes that the wrist joint is fixed in space, and calculates the shoulder elevation angle such that the arm's shoulder is positioned in a plane perpendicular to the direction of the end-effector position.

The code then calculates the elbow angle(s) based on the position of the elbow joint. It calculates the distance between the shoulder and elbow joints and compares it to the sum of the lengths of the upper arm and forearm segments. If the distance matches the sum of the segment lengths, the elbow joint is positioned directly below the end-effector, and the IK solver can simply calculate the elbow angle based on the position of the end-effector. If the distance is different, there are two possible solutions for the elbow angle, which are both calculated by the solver.

Finally, the IK solver populates the output structure with the calculated joint angles, as well as a flag indicating whether each solution is valid or not. The solver may return multiple solutions, since there are multiple possible configurations of joint angles that can achieve a given end-effector position and orientation. The function returns a boolean value indicating whether the forward kinematics solution was successfully calculated.



