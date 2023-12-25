KENDO ROBOT- NTU CSIE5074 Robotics 2023/24 Final Project, Team 5
=======================================
This is the final project repo of the course CSIE5074 (robotics 2023-24) at national Taiwan university (NTU).

We built a supervisory teleoperation kendo robot system. The robot is able to track the opponent’s pose and wait for the operator’s command to attack.

For detection and tracking the human body, we utilise a deep-learning-based human pose estimation algorithm (LOGO-CAP, CVPR 2022). Merging the RGB and DEPTH image of RealSense D435 together, we obtain the 3D coordinate of human body.

An interactive GUI is built using JavaScript for visualisation and real-time implementation. After ONE CLICK on the GUI button of the operator, the attack command will be sent to ROS (through web socket) and control the robot arm.

The GUI is hardcrafted based on ganga.js, which is a web programmable platform created by the geometric algebra community.

The visualisation, inverse kinematics and trajectory planning are all solved using conformal geometric algebra (CGA), which is a powerful mathematic framework for representing geometry. CGA can provide numerous geometric insights of the robot that are nearly impossible to be done in traditional matrix methods.

* Contact us
    #. Wei-Hsuan Cheng (team leader): r11631045@ntu.edu.tw
    #. Wen Perng: b10901042@ntu.edu.tw
    #. Che-Jung Chuang: r12021008@ntu.edu.tw
    #. Cheng-Yen Yu: r12922135@ntu.edu.tw

* `YouTube demo video <https://www.youtube.com/watch?v=9ygvFLr1BNQ>`_

* `Interactive GUI of kendo robot system <https://enkimute.github.io/ganja.js/examples/coffeeshop.html#ZAxvNkQ7x>`_

* `LOGO-CAP human pose estimation algorithm <https://github.com/cherubicXN/logocap>`_

* The geometric algebra community- `biVector.net <https://bivector.net/>`_

* `ganja.js <https://github.com/enkimute/ganja.js?files=1>`_ 
