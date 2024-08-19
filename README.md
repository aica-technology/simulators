# Simulators

An AICA application consists of various components and controllers that execute a specific behavior or a sequence of 
behaviors. These behaviors can quickly become complex, making it challenging to evaluate their safety and functionality
 without real hardware. Simulators offer an ideal solution for this issue. They can emulate the physics affecting the 
 robot and demonstrate the behavior of an AICA application without the need to connect to actual robots.

In this repository, we provide several simulator options that enable users to visualize the behavior of the robot while 
running an AICA application. These include the official UR Simulator, available in a Docker image, and the MuJoCo Simulator.

The UR Simulator is the official tool provided by Universal Robots for simulating their robots. It closely mimics the 
actual pendant used by UR, providing a realistic experience that closely replicates how a user would interact with the 
actual robot. 

MuJoCo is a free and open-source physics engine that simulates the dynamics of a robot. MuJoCo simulator is more general 
compared to UR Simulator as it is not related to one robot brand.


# How to connect to a simulator

Connecting to a simulator in an AICA application is extremely straightforward. In AICA Studio, users can select a URDF 
for a specific simulator. For example, as shown in the image below, users can choose the corresponding URDF from the 
dropdown menu. This selection is related to a specific hardware driver, which also includes the simulator driver.

![URDF Selection](./images/urdf_selection.png)

This makes it extremely easy for the user to switch between a simulated robot driver and an actual robot driver.