# Releases

Version history for this repository (4 releases).

## v1.0-release: v1.0-release
**Published:** 2021-12-22

-OpenCV static matrices changed to Eigen matrices. The average code speed-up is 16% in tracking and 19% in mapping, w.r.t. times reported in the ORB-SLAM3 paper.
-New calibration file format, see file Calibration_Tutorial. Added options for stereo rectification and image resizing.
-Added load/save map functionalities.
-Added examples of live SLAM using Intel Realsense cameras.
-Fixed several bugs.

[View on GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3/releases/tag/v1.0-release)

---

## v0.4-beta: v0.4-beta
**Published:** 2021-09-29
**Pre-release**

V0.4: Beta version

This version of the code used to obtain the experimental results reported in the 
TRO paper [1].

- Changed OpenCV dynamic matrices to OpenCV static matrices to speed up the code.

- Capability to measure running time of the system threads.

- Compatibility with OpenCV 4.0 (Requires at least OpenCV 3.0)

- Fixed minor bugs.

> [1] C. Campos, R. Elvira, J. J. G. Rodríguez, J. M. M. Montiel and J. D. Tardós, "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual–Inertial, and Multimap SLAM," in IEEE Transactions on Robotics, doi: 10.1109/TRO.2021.3075644. [PDF](https://arxiv.org/pdf/1610.05949.pdf)


[View on GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3/releases/tag/v0.4-beta)

---

## v0.3-beta: v0.3-beta
**Published:** 2020-09-07
**Pre-release**

- RGB-D compatibility, the RGB-D examples had been adapted to the new version.

- Kitti and TUM dataset compatibility, these examples had been adapted to the new version.

- ROS compatibility, It had been updated the old references in the code to work with this version.

- Config file parser, the YAML file contains the session configuration, a wrong parametrization may break the execution without any information to solve it. This version parses the file to read all the fields and give a proper answer if one of the fields have been wrong deffined or don't exist.

- Fixed minor bugs.

[View on GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3/releases/tag/v0.3-beta)

---

## v0.2-beta: v0.2-beta
**Published:** 2020-07-30
**Pre-release**



[View on GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3/releases/tag/v0.2-beta)

---

