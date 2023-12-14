# mujoco simulator

## To run
`python3 scripts/mujoco_simulator.py`

## To load pick and place configuration
Currently, we can print forces and torques by setting debug_forcetorque to true in mujoco_simulator.py.

The debug prints: message number; forces in x,y,z; torques about x,y,z.

## Quirks
While all axes look different between the MuJoCo .xml and the urdf file in rviz, joint coordinates are universal and the robot performs as commanded.

However, the sensor_frame is aligned with the tool frame in the urdf/rviz, and it is represented as the dummy body "sensor_frame".

## Generate your own shapes
Currently, I have not tried generating a non-primitive shape for the pick/place object at the end effector. Doing this would be slightly more complicated than receptacle as one must edit the ur5e xml file in a non-trivial way.

However, to generate the receptacle:
1. Generate your obj file using a CAD software of your choice.
2. Copy the obj file into a folder. This folder should only contain the obj file you wish to convert to a mujoco asset.
3. Install obj2mjcf and v-hacd. obj2mjcf has a script to install v-hacd. https://github.com/kevinzakka/obj2mjcf
4. `obj2mjcf --obj-dir "__filepath__" --save-mjcf --compile-model --verbose --vhacd-args.enable --vhacd-args.max-output-convex-hulls XXX --vhacd-args.max-recursion-depth YYY` where XXX and YYY are integers.
5. Copy all files from the newly generated folder into the assets folder in the same directory as the robot xml file.
6. For the ur5e.xml, two default groups named `visual` and `collision` are already in use. In our receptacle xml file generated, we can comment out `visual` since it has the same information and find and replace `collision` with `collision_rc`.
7. Modify the main body in the xml file to move the receptacle to the right location. See below for an example. The position of the end effector can be found from site xpos in MJDATA.
```
<worldbody>
    <body name="receptacle_cylinder" pos="-0.44 0.145 0">
```
8. Add `<include file="assets/receptacle_cylinder.xml"/>`, for example, at the top of the scene.xml file to include it

### Putting large number of obj files in a folder
To do this, the include tag needs to be updated. Also, it seems mujoco only accepts 1 mesh directory so if there are multiple directories, then you will have to specify relative to the meshdir. For example `<mesh file="receptacle_cylinder/receptacle_cylinder.obj"/>`.