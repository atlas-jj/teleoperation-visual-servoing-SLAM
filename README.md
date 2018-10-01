# teleoperation-visual-servoing-SLAM


#### Robot Fine Manipulation for Tele-operation under Time-delay Network Conditions

#### From coarse to fine manipulation: a Teleoperation System using uncalibrated Vision Guidance

> Abstract— We present a vision guided semi-autonomous tele-operation system using a coarse-to-fine manipulation approach.
The system is easy to setup. It only needs a low cost webcam that is freely mounted on the robot arm, and no prior camera
robot calibration or hand-eye calibration is needed. By using a real-time 3D reconstruction method based on our previous
work [1], a coarse 3D geometric model of the remote scene is generated and then further used in task specification.Our
proposed system begins with an initialization process, which can generate a coarse 3D geometric model of the remote scene.
The generated 3D model is then used for task specification which can remove the common constraint in vision guided
teleoperation system that target must be in camera’s field of view.teleoperation system that target must be in camera’s field of view.

# System Overview
![system](https://github.com/atlas-jj/teleoperation-visual-servoing-SLAM/blob/master/media/Design_Overview.jpg?raw=true)
                
				
#### TODO list

- [x] Orb_SLAM + CARV (repository in gitlab)
- [x] wam_ orbSLAM_routine
- [x]  wam_orbSLAM_Solver
- [x]  wam_pbvs
- [ ] wam_ibvs_Uncalibrated
- [ ] GUI
    - [ ] Task Specification (coarse)
    - [ ] Task Specification (fine)
