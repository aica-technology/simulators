# simulators

## To run
`cd scripts`
`python3 mujoco_simulator.py`

## To load pick and place configuration
Scroll down to simulation on the left side, click *0* next to *Key*
Currently, we can print forces and torques by setting debug_forcetorque to true in mujoco_simulator.py.
The debug prints: message number; forces in x,y,z; torques about x,y,z.

## Closing MuJoCo
Ensure that the MuJoCo GUI is closed before interrupting the program (ctrl-C). There is no graceful shutdown (yet).

## Quirks
While all axes look different between the MuJoCo .xml and the urdf file in rviz, joint coordinates are universal and the robot performs as commanded.
However, the sensor_frame is aligned with the tool frame in the urdf/rviz, and it is represented as the dummy body "sensor_frame".