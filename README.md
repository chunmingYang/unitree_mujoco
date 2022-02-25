# unitree_mujoco
Unitree A1 working with MuJoCo 2.0

Working Log, see [Docs](https://docs.google.com/document/d/13MPEeioKg0B8ffl0e8jVBjJx07-_Jb92d5Q8VLzRPng/edit) & [Slides](https://docs.google.com/presentation/d/10Jx7j-Y9MVSetwgpemRar8u9XWPIoCmTDSeaENsuK6Q/edit#slide=id.p)




###### Branch List Description
- ctrl_00: Original MuJoCo files
- ctrl_01: Simple simultaneous PD control of four legs
- ctrl_02: Template with foot position (x/y/z) sensors, which also save mjData doing trot gait
- ctrl_03: Template with shoulder, elbow, foot position (x/y/z) sensors\
&emsp;&emsp;&emsp;&ensp; Save mjData\
&emsp;&emsp;&emsp;&ensp; mocap and equality constraint fix the trunk
- ctrl_04: Template with shoulder, elbow, foot position (x/y/z) sensors\
&emsp;&emsp;&emsp;&ensp; Save mjData\
&emsp;&emsp;&emsp;&ensp; mocap and equality constraint fix the trunk\
&emsp;&emsp;&emsp;&ensp; IK using jacobian approach\
&emsp;&emsp;&emsp;&ensp; IK using Newton-Raphson method to solve FW equations\
&emsp;&emsp;&emsp;&ensp; IK using analytic method to solve FW equations

