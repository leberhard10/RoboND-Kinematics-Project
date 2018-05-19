## Project: Kinematics Pick & Place
#### Submitted by Laura Eberhard

---

**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points 

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

This file will cover the rubric criteria and how item was addressed. The default README file was also altered to include a disclaimer about Udacity's honor code, since the project is in a public repository on github.

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

To determine the DH parameters, a diagram of the Kuka KR210 joints and links was created and the Z-axis of each joint was determined. 

![arm_fig_1][https://github.com/leberhard10/RoboND-Kinematics-Project/tree/master/misc_images/X-Axis.png]

At first each axis was placed with each joint, but it was realized that it would result in an increase in number of calculations. Since the course mentioned that axis can be moved to decrease the number of DH parameters, a new diagram was created. If an axis didn't change between between joints, the x or z axis for those joints were made collinear to simplify the parameter table.

![arm_fig_2][https://github.com/leberhard10/RoboND-Kinematics-Project/tree/master/misc_images/RobotArm.jpg]

After watching more videos and attempting to determine how the project kinematics 1 video assigned the axis locations. It was decided to continue with the diagram shown in the Project KR210 Forward Kinematics videos. The diagram below lists the axis along with the thetas for each joint.

![arm_fig_3][https://github.com/leberhard10/RoboND-Kinematics-Project/tree/master/misc_images/RobotArm1.jpg]

The base (0) and joint 1 z-axis are collinear, while joints 2 and 3 are parallel. Joint 6 is also coliniear with the gripper (EE) z-axis. According to section 13 of the Forward and Inverse Kinematics lesson, collinlear lines in the Z axis mean alpha = 0 and a = 0. Parallel axis mean alpha is 0 and a is not 0. Intersecting Z axis will meant that alpha is not 0 and a = 0. Alpha values provided by the KR210 Forward Kinematics 1 video.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i) | Notes
--- | --- | --- | --- | --- | ---
0->1 | 0 | 0 | ? | ? | Colinear: alpha & a = 0
1->2 | -90 | a1 | ? | ? | 
2->3 | 0 | a2 | ? | ? | Parallel: alpha = 0
3->4 | -90 | a3 | ? | ? | 
4->5 | 90 | 0 | ? | ? | Interset: a = 0
5->6 | -90 | 0 | ? | ? | Intersect: a = 0
6->G | 0 | 0 | ? | ? | Colinear: alpha & a = 0

The video in KR210 Forward Kinematics 1, shows which a's and d's need to be solved for with the DH table and supply the remaining alpha values. It is assumed that the rules of calculating d are the same as a, but with the X axis.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i) | Notes
--- | --- | --- | --- | --- | ---
0->1 | 0 | 0 | d1 | ? |  Parallel: d != 0
1->2 | -90 | a1 | 0 | ? | Intersect: d = 0
2->3 | 0 | a2 | 0 | ? | Colinear: d = 0 
3->4 | -90 | a3 | d4 | ? |  Parallel: d != 0
4->5 | 90 | 0 | 0 | ? | Colinear: d = 0 
5->6 | -90 | 0 | 0 | ? | Colinear: d = 0 
6->G | 0 | 0 | dG | ? | Parallel: d != 0

The a and d variables were added to the diagram as seen below.

![arm_fig_4][https://github.com/leberhard10/RoboND-Kinematics-Project/tree/master/misc_images/RobotArm2.jpg]


Finally, each of the joints are revolute, so the table would result in solving for each theta.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i) | Notes
--- | --- | --- | --- | --- | ---
0->1 | 0 | 0 | d1 | Th1 |  
1->2 | -90 | a1 | 0 | Th2 | 
2->3 | 0 | a2 | 0 | Th3 | 
3->4 | -90 | a3 | d4 | Th4 |  
4->5 | 90 | 0 | 0 | Th5 | 
5->6 | -90 | 0 | 0 | Th6 |  
6->G | 0 | 0 | dG | ThG | 



#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2

The alpha values were converted to radians following the example table.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | d1 | Th1 |  
1->2 | - pi/2 | a1 | 0 | Th2 | 
2->3 | 0 | a2 | 0 | Th3 | 
3->4 | - pi/2 | a3 | d4 | Th4 |  
4->5 | pi/2 | 0 | 0 | Th5 | 
5->6 | - pi/2 | 0 | 0 | Th6 |  
6->G | 0 | 0 | dG | ThG | 

As specified in the KR210 Forward Kinematics 2 video, the transformation between the base link and the gripper link is a 180 degree rotation about the z axis and -90 degrees on the y axis.


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  



