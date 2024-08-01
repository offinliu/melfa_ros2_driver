<img src="./doc/figures/melfa_logo.png" width="238" height="48"> <img src="./doc/figures/ros2_logo.png" width="200" height="48">

# __MITSUBISHI ELECTRIC INDUSTRIAL ROBOT MELFA__
    
## __1. Overview__

Introducing the next generation of intelligent robots, incorporating advanced solutions technology and “e-F@ctory”, technologies and concepts developed and proven using Mitsubishi Electric’s own production facilities that go beyond basic robotic performance to find ways of reducing the TCO in everything from planning and design through to operation and maintenance. 
<br>

[![RTR demo](https://markdown-videos.vercel.app/youtube/X7CBorolqrE?si=VlLh2xOvs8k0HvZc)](https://youtu.be/X7CBorolqrE?si=VlLh2xOvs8k0HvZc)
[![MEAU demo](https://markdown-videos.vercel.app/youtube/Ks6ji6kw68c?si=KCWWB8-P_m3ofB4V)](https://youtu.be/Ks6ji6kw68c?si=KCWWB8-P_m3ofB4V)
- [Learn more](https://www.mitsubishielectric.com/fa/products/rbt/robot/index.html)
- [Robot catalog](https://dl.mitsubishielectric.com/dl/fa/document/catalog/robot/l(na)-09091eng/l09091m.pdf)

<br/>

MELFA ROS2, co-developed with [ROS-Industrial Consortium Asia Pacific](https://rosindustrial.org/ric-apac), provides a suite of tools to enable the creation of advance solutions using our industry proven platform. Mitsubishi Electric provides a ros2 driver, ros2 io controllers, robot description files and moveit_config packages of each robot; optimized in-house by our developers to ensure high performance.

## __2. MELFA ROS2 Feature__

MELFA ROS2 consist of six main componets: melfa_bringup, melfa_description, melfa_driver, melfa_io_controllers, melfa_msgs, various moveit_config packages.

### __melfa_bringup__

- provides launch files for robot bringup

### __melfa_description__

- contains robot descriptions
- ros2_controllers

### __melfa_driver__

- supports [ros2_control](https://control.ros.org/humble/doc/getting_started/getting_started.html).
- provides __real time communication__<sup>1</sup> hardware interface with our CR800-R/Q/D robot controllers via __rtexc api__ <sup>2</sup> from our [__MELFA ethernet SDK__](https://github.com/Mitsubishi-Electric-Asia/melfa_ethernet_sdk/tree/main). 
- connects to the robot controller via __rtexc api__ to control the robot via __MELFA BASIC VI__<sup>3</sup> __MXT__<sup>4</sup> command. The robot position command, robot state & I/O data are transmitted through this connection. 
- includes quality of life features built into __rtexc api__ such as user configurable disconnection detection and debugging tools.

### __melfa_io_controllers__

- supports [ros2_control](https://control.ros.org/humble/doc/getting_started/getting_started.html).
- user configurable io controllers.
- provides ROS2 controllers for GPIO control

### __melfa_msgs__

- provides ROS2 msgs for MELFA robots

### __melfa_robot-model_moveit_config__

- provides example MoveIt config and launch files for MELFA robots
- supports OMPL, Pilz Industrial Planner, CHOMP and Moveit servo.
- optimized by our developers to ensure high performance in speed and accuracy.


&#10146; <sup>1</sup>  __real time communication__ frequency is 286Hz for CR800-R & CR-800-D and 141Hz for CR800-Q.

&#10146; <sup>2</sup>  __rtexc api__ stands for Real Time External Control API.

&#10146; <sup>3</sup>  __MELFA BASIC VI__ is our proprietary robot programming language.

&#10146; <sup>4</sup>  __MXT__ is the command to enable __real time external control__.

>Note1: You can download the [Ethernet Function Instruction Manual](https://dl.mitsubishielectric.co.jp/dl/fa/document/empf/manual/robot/bfp-a3379/bfpa3379g.empf) from [Robot Industrial/Collaborative Robot MELFA Manual](https://www.mitsubishielectric.co.jp/fa/download/search.do?mode=manual&kisyu=/robot).<br/>


## __3. MELFA ROS2 Usage and Installation__

MELFA ROS2 is designed to interfce CR800 robot controllers with the ROS2 so that developers can leverage the contributions from the Open Source Community with an industry proven robot platform.
<br/>

&#10148; If your ROS PC is installed with ROS 2 Humble Hawksbill, see [MELFA ROS2-humble](https://github.com/Mitsubishi-Electric-Asia/melfa_ros2/tree/humble) and follow the user guide.<br/>

- [MELFA ROS2 user guide](./doc/melfa_ros2_humble.md) : Usage and Installation of MELFA ROS2.
- [RT Toolbox3 Setup](./doc/rt_toolbox3_setup.md) : Create your first RT Toolbox3 Project File for ROS2.
- [RT Toolbox3 Simulator Setup](./doc/rt_sim_setup.md) : Connect to RT Toolbox3 simulator as if it is a real robot.
- [RT Toolbox3 Real Robot Setup](./doc/rt_real_setup.md): Connect to a MELFA robot.

  
<div> </div>

## __4. Other MELFA ROS2 Related Repositories__

- [MELFA ROS2 8axis with Demo](https://github.com/Mitsubishi-Electric-Asia/melfa_ros2_8axis/tree/humble) : ROS2 description, moveit_config and simple moveit application for 6-axis articulated robot with 2-axis travel axis. Accompanied with RT Toolbox3 Project File to try in RT Toolbox3 simulator.
- [MELSOFT Simulators Demo](https://github.com/Mitsubishi-Electric-Asia/melsoft_simulators_demo/tree/humble) : uses MELSOFT GX simulator3, MELSOFT GT simulator3 and MELSOFT RT Toolbox3 simulator to demostrate "accurate" simulations using proprietary technologies that can be leveraged by the open source community.
- [Simple applications](https://github.com/Mitsubishi-Electric-Asia/simple_moveit_scripts/tree/humble) : experience MELFA ROS2 features through simple applications.

<div> </div>

## __5. MELFA Naming Convention__

This section provides a brief introduction to naming conventions of MELFA robots. Below are images from our [robot catalog](https://dl.mitsubishielectric.com/dl/fa/document/catalog/robot/l(na)-09091eng/l09091m.pdf) describing the naming convention.

For articulated robots (RV), it is fairly straigth forward as the variations that contribute to package differences are __Maximum load capacity__, __Series__ and __Arm length__. 

<br/>

<img src="./doc/figures/naming_convention_rv.png" width="1000" heigth="500" >

<br/>

For SCARA robots (RH), it is has more variations that contribute to packages differences such as __Maximum load capacity__, __Series__, __Arm length__ in cm and __Vertical stroke__ in cm.

<br/>

<img src="./doc/figures/naming_convention_rh.png" width="1000" heigth="500" >

<br/>

__Environment specifications__, __Internal wiring__ and __Controller type__ do not contribute to kinematic variations. However, it is important to take note of __Controller type__ as it may change the __Control frequency__ and/or __I/O controller__ settings.


## __6. Contact us / Technical support__
More Support & Service, please contact us [@MEAP](https://sg.mitsubishielectric.com/fa/en/contact.html) or send us an email [@Dev team](muyao.liu@asia.meap.com) for development related enquiries.

<div> </div>