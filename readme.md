# KENDO ROBOT- Combining Robotics Technologies with Martial Art
NTU CSIE5074 Robotics 2023/24 Final Project, Team 5

This GitHub repository contains the code files and report for the final project competition in the course [CSIE5074 (Robotics 2023-24)](https://nol.ntu.edu.tw/nol/coursesearch/print_table.php?course_id=922%20U1070&class=&dpt_code=9210&ser_no=62682&semester=112-1&lang=CH) at [National Taiwan University (NTU)](https://www.ntu.edu.tw/english/). Click below image to watch our demo video!
<div align="center">
<a href="https://www.youtube.com/watch?v=9ygvFLr1BNQ" target="_blank"><img src="[https://github.com/wei-hsuan-cheng/kendo_robot/Pictures/video_cover.jpg](https://github.com/wei-hsuan-cheng/kendo_robot/blob/main/Pictures/video_cover.jpg)" alt="video" width="48%" /></a>
</div>

We built a supervisory teleoperation kendo robot system. The robot is able to track the opponent’s pose and wait for the operator’s command to attack.

To detect and track the human body, we utilized a deep-learning-based human pose estimation algorithm ([LOGO-CAP, CVPR 2022](https://github.com/cherubicXN/logocap)). By merging the RGB and DEPTH images from the RealSense D435, we obtained the 3D coordinates of the human body.

An interactive GUI was created using `JavaScript` for visualization and real-time implementation. With just one click on the GUI button, the operator can send an attack command to ROS (via a WebSocket), which controls the robot arm.

The GUI is handcrafted based on `ganja.js`, a web-programmable platform developed by the geometric algebra community.

Visualization, inverse kinematics, and trajectory planning are all handled using conformal geometric algebra (CGA), a powerful mathematical framework for representing geometry. CGA provides numerous geometric insights into the robot that are nearly impossible to achieve using traditional matrix methods.

## Contact Us

- **Wei-Hsuan Cheng** (Team Leader): r11631045@ntu.edu.tw
- **Wen Perng**: b10901042@ntu.edu.tw
- **Che-Jung Chuang**: r12021008@ntu.edu.tw
- **Cheng-Yen Yu**: r12922135@ntu.edu.tw

## Resources

- [YouTube Demo Video](https://www.youtube.com/watch?v=9ygvFLr1BNQ)
- [Interactive GUI of Kendo Robot System](https://enkimute.github.io/ganja.js/examples/coffeeshop.html#ZAxvNkQ7x)
- [LOGO-CAP Human Pose Estimation Algorithm](https://github.com/cherubicXN/logocap)
- [The Geometric Algebra Community - biVector.net](https://bivector.net/)
- [ganja.js](https://github.com/enkimute/ganja.js?files=1)
