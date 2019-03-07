## Project: Kinematics Pick & Place
This writeup describes how the DH parameter table and transform matrices are obtained,  and the derivation of each joint angles. And finally how `IK_server.py` is implemented using method above.

[/]: # "Image References"

[image1]: ./misc_images/DH.jpg
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[tfm1]: ./misc_images/tfm1.png
[tfm2]: ./misc_images/tfm2.png
[eular]: ./misc_images/eular.png
[kr210]: ./misc_images/kr210.jpg
[pnp1]: ./misc_images/PnP1.jpg
[pnp2]: ./misc_images/PnP2.jpg
[pnp3]: ./misc_images/PnP3.jpg
[pnp4]: ./misc_images/PnP4.jpg
[pnp5]: ./misc_images/PnP5.jpg
[pnp6]: ./misc_images/PnP6.jpg

***



###Kinematic Analysis

#### 1. DH Parameter Table

Follow the parameter assignment process for open kinematic chains with 6 degrees of freedom, I come out with figure below, and the alphas are listed: 

![alt text][image1]



Through checking the KR210.urdf.xacro file under urdf folder, the relative location of each joint can be summarized as:

| Joint Name    | Parent Link | Child Link   | x     | y    | z      | roll | pitch | yaw  |
| ------------- | ----------- | ------------ | ----- | ---- | ------ | ---- | ----- | ---- |
| joint_1       | base_link   | link_1       | 0     | 0    | 0.33   | 0    | 0     | 0    |
| joint_2       | link_1      | link_2       | 0.35  | 0    | 0.42   | 0    | 0     | 0    |
| joint_3       | link_2      | link_3       | 0     | 0    | 1.25   | 0    | 0     | 0    |
| joint_4       | link_3      | link_4       | 0.96  | 0    | -0.054 | 0    | 0     | 0    |
| joint_5       | link_4      | link_5       | 0.54  | 0    | 0      | 0    | 0     | 0    |
| joint_6       | link_5      | link_6       | 0.193 | 0    | 0      | 0    | 0     | 0    |
| gripper_joint | link_6      | gripper_link | 0.11  | 0    | 0      | 0    | 0     | 0    |
|               |             | TOTAL        | 2.153 | 0    | 1.946  |      |       |      |



Thus, DH table below could be completed with the a's and d's value from table above:

| Links | alpha(i-1) | a(i-1) | d(i-1) |  theta(i)  |
| :---: | :--------: | :----: | :----: | :--------: |
| 0->1  |     0      |   0    |  0.75  |     qi     |
| 1->2  |   - pi/2   |  0.35  |   0    | -pi/2 + q2 |
| 2->3  |     0      |  1.25  |   0    |     qi     |
| 3->4  |   - pi/2   | -0.054 |  1.5   |     qi     |
| 4->5  |    pi/2    |   0    |   0    |     qi     |
| 5->6  |   - pi/2   |   0    |   0    |     qi     |
| 6->EE |     0      |   0    | 0.303  |     0      |

Note that there is always a constant of -90 degree between X1 and X2 (theta2).

#### 2. Transform Matrices

DH convention uses four individual transform, 

![alt text][tfm1] 

to describe the relative translation and orientation of link (i-1) to link (i). In matrix form, this transform is,

![alt text][tfm2]

where c = cos(), s = sin().

From the DH parameter in previous section and the transform matrix above, following homogeneous transform matrices are derived

 ```
def TF_Matrix(alpha, a, d, q):
	TF = Matrix([[			cos(q),			  -sin(q), 			 0,	a],
	 [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
	 [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
	 [				 0,					0,			 0,			1]])
	return TF

# Create individual transformation matrices
T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
T6_EE= TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
T0_EE= T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
 ```



#### 3. Joint Angles

Exact the Wrist Center `WC` and find out the joint angles geometric IK method:

![alt text][kr210]

With the figure above, `theta1` is derived:

```
theta1 = atan2(WC[1], WC[0])
```

![alt text][image2]

From figure above, derive `theta2` and `theta3`:

```
side_a = 1.501
side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
side_c = 1.25

angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
theta3 = pi / 2 - (angle_b + 0.036)
```



Once the first three joint variables are known, calculate `R0_3` via application of homogeneous transforms up to the WC:

```
R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
```

Then, find a set of Euler angles corresponding to the rotation matrix:

![alt text][eular]

```
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
R3_6 = R0_3.inv("LU") * ROT_EE
```



Finally, with the `R3_6`, derive `theta4`, `theta5` and `theta6`:

```
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```

***



### Project Implementation

In `IK_server.py` , at the beginning of  `handle_calculate_IK(req)`, function will check the validity of `req` content. If invalid, no action taken, else function will process it and return `CalculateIKResponse(joint_trajectory_list)`. 

First, create symbols `d1-d7`, `a0-a6`, `alpha0-alpha6` and `q1-q7` using `symbols()` function, where `d` is link offset, `a` is link length, `alpha` is twist angle and `q` is joint address.

Then, define `DH_Table` with the parameter shown in table above:

```
DH_Table = {alpha0:	  0,    a0:      0, d1:  0.75, q1:		   q1,
		   alpha1: -pi/2., a1:   0.35, d2:     0, q2: -pi/2. + q2, 
		   alpha2: 	  0,   a2:   1.25, d3:     0, q3:          q3,
		   alpha3: -pi/2., a3: -0.054, d4:   1.5, q4:          q4,
		   alpha4:  pi/2., a4:      0, d5:     0, q5:          q5,
		   alpha5: -pi/2., a5:      0, d6:     0, q6:          q6,
		   alpha6:      0, a6:      0, d7: 0.303, q7:           0 }
```

then, create transform matrices as mentioned in previous section, and followed by rotation matrices:

```
r, p, y = symbols('r p y')

ROT_x = Matrix([[	   1, 	     0,		  0],
				[	   0,   cos(r),	-sin(r)],
				[	   0,	sin(r),	 cos(r)]])
		
ROT_y = Matrix([[  cos(p), 	     0,	 sin(p)],
				[	    0,  	 1,		  0],
				[ -sin(p),		 0,	 cos(p)]])

ROT_z = Matrix([[  cos(y), -sin(y),	      0],
      		   [  sin(y),  cos(y),		  0],
			   [       0,		 0,	      1]])

ROT_EE= ROT_z * ROT_y * ROT_x
```



After exact end-effector position and orientation from `req` , calculate the compensate for rotation discrepancy between DH parameters and Gazebo, and get the wrist center `WC`:

```
Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
ROT_EE = ROT_EE * Rot_Error
ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

EE = Matrix([[px],
			 [py], 
			 [pz]])

WC = EE - (0.303) * ROT_EE[:,2]
```

Lastly, calculate the joint angles using Geometric IK method as mentioned in previous section.



Here are the screenshots demonstrating the actions taken by KR210 in retrieving the blue object and drop it to drop-of point:

| Step description                                  |    Screenshot     |
| ------------------------------------------------- | :---------------: |
| Moving to the target object location              | ![alt text][pnp1] |
| Reach target location                             | ![alt text][pnp2] |
| Grasping target object & Retrieving target object | ![alt text][pnp3] |
| Moving to drop-off point                          | ![alt text][pnp4] |
| Reach drop-off point                              | ![alt text][pnp5] |
| Release target object                             | ![alt text][pnp6] |