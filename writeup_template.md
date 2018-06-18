## Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/IMG_3284.jpg
[image5]: ./misc_images/IMG_3283.jpg
[image6]: ./misc_images/image-3.png
[image7]: ./misc_images/image-4.png
[image8]: ./misc_images/IMG_3285.JPG
[image9]: ./misc_images/image-3_1.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Values for link length and offset were taken from the joint origins listed in kr210.urdf.xacro.

The pi/2 offsets were determined by launching the forward_kinematics demo and comparing the zero angle to the diagram for the DH parameters below.

![alt text][image5]

a1 = 0.35
d1 = 0.33 + 0.42
a2 = 1.25
a3 = -0.054
d4 = 0.96 + 0.54
d7 = 0.193 + 0.11

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | d1 | q1
1->2 | - pi/2 | a1 | 0 | q2 - pi/2
2->3 | 0 | a2 | 0 | q3
3->4 |  -pi/2 | a3 | d4 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | d7 | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The transform from lesson 11.12 is ![alt text][image6]
Sympy was used to make the individual transforms from the base link to the gripper. Applying the DH table to each transform gives the following:

t0_1:

|||
--- | --- | --- | ---
cos(q1) | -sin(q1) | 0 | 0
sin(q1) | cos(q1) | 0 | 0
0 | 0 | 1 | 0.75
0 | 0 | 0 | 1

t1_2:

|||
--- | --- | --- | ---
sin(q2) | cos(q2) | 0 | 0.35
0 | 0 | 1 | 0
cos(q2) | -sin(q2) | 0 | 0
0 | 0 | 0 | 1

t2_3:

|||
--- | --- | --- | ---
cos(q3) | -sin(q3) | 0 | 1.25
sin(q3) | cos(q3) | 0 | 0
0 | 0 | 1 | 0
0 | 0 | 0 | 1

t3_4:

|||
--- | --- | --- | ---
cos(q4) | -sin(q4) | 0 | -0.054
0 | 0 | 1 | 1.50
-sin(q4) | -cos(q4) | 0 | 0
0 | 0 | 0 | 1

t4_5:

|||
--- | --- | --- | ---
cos(q5) | -sin(q5) | 0 | 0
0 | 0 | -1 | 0
sin(q5) | cos(q5) | 0 | 0
0 | 0 | 0 | 1

t5_6:

|||
--- | --- | --- | ---
cos(q6) | -sin(q6) | 0 | 0
0 | 0 | 1 | 0
-sin(q6) | -cos(q6) | 0 | 0
0 | 0 | 0 | 1

t6_7:

|||
--- | --- | --- | ---
1 | 0 | 0 | 0
0 | 1 | 0 | 0
0 | 0 | 1 | 0.303
0 | 0 | 0 | 1

To get the homogeneous transform from base to gripper given the gripper orientation and position is

![alt text][image9]

Px, Py, Pz are the co-ordinates of the end effector.

The rotation matrix is compound rotation of roll, pitch, yaw of the end effector plus a correction from the DH parameter world to the Gazebo world.

```python
rotation_intrinsic = rotation_z * rotation_y * rotation_x
rotation_correction = rotation_z.subs(y, pi) * rotation_y.subs(p, -pi/2)
```
The resulting rotation matrix R0_6 is:

|||
--- | --- | --- | ---
sin(p) x cos(r) x cos(y) + sin(r) x sin(y) | -sin(p) x sin(r) x cos(y) + sin(y) x cos(r) | cos(p) x cos(y)
sin(p) x sin(y) x cos(r) - sin(r) x cos(y) | -sin(p) x sin(r) x sin(y) - cos(r) x cos(y) | sin(y) x cos(p)
cos(p) x cos(r) | -sin(r) x cos(p) | -sin(p)

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The approach for IK is to determine the wrist centre given end effector position and angle.
```python
wrist_centre = end_effector - d7 * rotation_ee[:,2]
```
![alt text][image4]

Once the wrist centre is determined, the first 3 joint angles can be determined.
The only joint affecting z-axis rotation is the first joint, so ignoring the z component yields the angle.
```python
theta1 = atan2(wrist_centre[1], wrist_centre[0])
```

The 2nd angle is determined by making a triangle of joint 2, 3, and 5 and then solving for the angle using cosine law.

![alt text][image8]

The 3rd angle is likewise determined using cosine law except that angle has to be adjusted because the 3rd side of the triangle between joint 3 and 5 doesn't include joint 4. The correction is:
```python
theta3_correction  =  atan2(abs(a3), d4)
```


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  

The code is based on the walk-through of IK_debug.py.

A lot of people had an issue using the inverse matrix function, so I'm using transpose instead for determining R3_6.

The link shows the Kuka slowly but successfully picking and placing 8 out of 10 items. The gripper seems to randomly fail to pick up items even though the end effector seems to be accurately located. The arm also drops an item because it bangs the item into the shelf.

https://www.youtube.com/watch?v=mcbETRq7XiM&t=618s

(best to watch on 2x speed)

The arm is a little jittery, but a numpy implementation could improve response time. There are also lots of unnecessary movements of the wrist joint 3 and joint 5 because there are multiple solutions to the joint angles.
