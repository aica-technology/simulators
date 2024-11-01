"""
This program connects with controllertosim via localhost.
Publishes state information and subscribes control input to and from controllertosim respectively
Launches MuJoCo viewer and visualises robot state
"""
import os
import signal
import sys
import time

import clproto
import mujoco
import mujoco.viewer
from communication_interfaces.sockets import (ZMQCombinedSocketsConfiguration,
                                              ZMQContext, ZMQPublisher,
                                              ZMQPublisherSubscriber,
                                              ZMQSocketConfiguration)
from state_representation import CartesianWrench, JointState


def signal_handler(sig, frame):
    sys.exit(0)


class Simulator:
    def __init__(self, xml_path: str, export_ft=False, debug_ft_interval=0):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self._context = ZMQContext()

        server_config = ZMQCombinedSocketsConfiguration(self._context, "*", "5001", "5002", True, True)
        self._wrench_pub = ZMQPublisher(ZMQSocketConfiguration(self._context, "*", "5003", True))
        self._server = ZMQPublisherSubscriber(server_config)
        self._wrench_pub.open()
        self._server.open()

        self._state = JointState().Zero("robot", ["ur5e_" + self.model.joint(q).name for q in range(self.model.nq)])
        self._wrench = CartesianWrench().Zero("tcp_fts_sensor")

        self._export_ft = export_ft
        self._debug_ft_interval = debug_ft_interval
        self._i = 0

        if self._export_ft:
            with open("force_torque_readings.txt", "w", encoding="utf-8") as file:
                file.write("fx,fy,fz,tx,ty,tz" + '\n')

    def control_loop(self, mj_model: mujoco.MjModel, mj_data: mujoco.MjData):
        for q in range(mj_model.nq):
            self._state.set_position(mj_data.qpos[q], q)
            self._state.set_velocity(mj_data.qvel[q], q)
        self._server.send_bytes(clproto.encode(self._state, clproto.MessageType.JOINT_STATE_MESSAGE))
        message = self._server.receive_bytes()
        if message:
            command = clproto.decode(message)
            if command:
                for u in range(mj_model.nu):
                    mj_data.ctrl[u] = command.get_velocity(u)

        force_torque_data = [*mj_data.sensor("ft_force").data, *mj_data.sensor("ft_torque").data]
        force_torque_data = [-x for x in force_torque_data]  # follow AICA convention for forces
        self._wrench.set_data(force_torque_data)
        self._wrench_pub.send_bytes(clproto.encode(self._wrench, clproto.MessageType.CARTESIAN_WRENCH_MESSAGE))

        if self._export_ft:
            with open("force_torque_readings.txt", "a", encoding="utf-8") as file:
                file.write("{},{},{},{},{},{}".format(*force_torque_data) + '\n')
        if self._debug_ft_interval > 0:
            self._i += 1
            if self._i % self._debug_ft_interval == 0:
                print(int(self._i/self._debug_ft_interval), *force_torque_data)


def main():
    script_dir = os.path.abspath(os.path.dirname(__file__))
    sim = Simulator(os.path.join(script_dir, os.pardir, "universal_robots_ur5e", "scene.xml"), False, 0)

    mujoco.set_mjcb_control(sim.control_loop)

    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        mujoco.mj_resetDataKeyframe(sim.model, sim.data, 0)
        while viewer.is_running():
            step_start = time.time()

            mujoco.mj_step(sim.model, sim.data)
            viewer.sync()

            # rudimentary time keeping, will drift relative to wall clock
            time_until_next_step = sim.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    main()
