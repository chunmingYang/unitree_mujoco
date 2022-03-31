# unitree_mujoco
Unitree A1 working with MuJoCo 2.0

###### Branch List Description
- ctrl_04: Template with FR_shoulder, FR_foot position (x/y/z)\
&emsp;&emsp;&emsp;&ensp; Save mjData\
&emsp;&emsp;&emsp;&ensp; mocap and equality constraint fix the trunk\
&emsp;&emsp;&emsp;&ensp; FR_IK using Newton-Raphson method to solve FW equations\
&emsp;&emsp;&emsp;&ensp; FR_IK using jacobian approach\
&emsp;&emsp;&emsp;&ensp; FR_IK using analytic method to solve FW equations\
&emsp;&emsp;&emsp;&ensp; matlab a1_fk.m for generating jacobian matrix for newton method\
&emsp;&emsp;&emsp;&ensp; matlab a1_analytic.m for analytical ik solution\
