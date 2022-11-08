from collections import OrderedDict

import numpy as np
import hydra
import omegaconf

from dmanus.dynamixel import dynamixel_py as dxl


class DManus:
    def __init__(self, ee_cfg: omegaconf.DictConfig):
        self.config = ee_cfg
        if "sensing" in ee_cfg:
            self.sensing = hydra.utils.instantiate(ee_cfg.sensing)
        else:
            self.sensing = None

        self._dxl_connected = False

        dxl_config = self.config.dynamixel

        self.active_motor_ids = dxl_config.active_motors

        self.reset_pos = dxl_config.reset_pos

        self._dxl_connected = self._connect_dxl(dxl_config)

        if not self._dxl_connected:
            print("Unable to connect to dynamixel motors")
            return

        self._engage_motors(True)

        self._ctrl_mode = dxl_config.control_mode

        self.torque_control_ids = [
            mid for mid, cmode in self._ctrl_mode.items() if cmode == "T"
        ]
        self.pos_control_ids = [
            mid for mid, cmode in self._ctrl_mode.items() if cmode == "P"
        ]

        self.active_tor_ids = np.intersect1d(
            self.torque_control_ids, self.active_motor_ids
        ).tolist()
        self.active_pos_ids = np.intersect1d(
            self.pos_control_ids, self.active_motor_ids
        ).tolist()

        self.active_motor_mask = [
            id in self.active_motor_ids for id in self._ctrl_mode.keys()
        ]

        self.pos_mask = [
            self._ctrl_mode[self.active_motor_ids[i]] == "P"
            for i in range(len(self.active_motor_ids))
        ]
        self.tor_mask = np.logical_not(self.pos_mask)

        self.pos_lims_high = np.array(
            list(dxl_config.motor_lims.pos_lims.high.values())
        )
        self.tor_lims_high = np.array(
            list(dxl_config.motor_lims.tor_lims.high.values())
        )
        self.pos_lims_low = np.array(list(dxl_config.motor_lims.pos_lims.low.values()))
        self.tor_lims_low = np.array(list(dxl_config.motor_lims.tor_lims.low.values()))

        # Set each motor individually to the appropriate mode
        if not dxl_config.cmodes_set:
            self.motors.torque_control(self.torque_control_ids, enable=True)
            self.motors.torque_control(self.pos_control_ids, enable=False)
        else:
            print("Control modes already set")
            for mid, cmode in self._ctrl_mode.items():
                self.motors.ctrl_mode[mid] = cmode == "T"

        # Flag to leave motors engaged when closing connection
        self._close_engaged = dxl_config.close_engaged

    def _connect_dxl(self, config) -> bool:
        if not self._dxl_connected:
            print("\nAttempting to connect dynamixels...")
            self.motors = dxl.dxl(
                motor_id=list(config.control_mode.keys()), **config.motor_params
            )
            self.motors.open_port()
            self.motor_ids = self.motors.motor_id

            print("Initialized Dynamixels\n")

        return True

    def _engage_motors(self, enable=True, motor_ids=None):
        if motor_ids == None:
            motor_ids = self.motor_ids
        self.motors.engage_motor(motor_ids, enable)

    def get_obs_limits(self):
        # TODO: Change this to be compatible with gym ie. change to boxes
        obs_low, obs_high = [], []
        if self.motors:
            obs_low.extend(self.motor_lims_low)
            obs_high.extend(self.motor_lims_high)

        return obs_low, obs_high

    def get_pos_limits(self):
        if self.motors:
            return self.pos_lims_low, self.pos_lims_high
        return [], []

    def get_tor_limits(self):
        if self.motors:
            return self.tor_lims_low, self.tor_lims_high
        return [], []

    def _get_raw_obs(self):
        obs_dict = OrderedDict()
        obs_dict["ee_q"] = self.motors.get_pos(self.motor_ids)
        obs_dict["ee_qdot"] = self.motors.get_vel(self.motor_ids)

        return obs_dict

    def get_obs(self):
        obs_dict = self._get_raw_obs()

        if self.sensing:
            rs_obs = self.sensing.get_obs()
            obs_dict.update(rs_obs)

        return obs_dict

    def reset(self):
        # TODO: Reset to specific position
        self.motors.torque_control(self.active_motor_ids, enable=False)
        self.motors.set_des_pos(self.active_motor_ids, self.reset_pos)

        self.motors.torque_control(self.torque_control_ids, enable=True)
        self.motors.torque_control(self.pos_control_ids, enable=False)

        obs, info = self._get_raw_obs(), {}
        if self.sensing:
            rs_obs, rs_info = self.sensing.reset()
            obs.update(rs_obs)
            info.update(rs_info)
        return obs, info

    def step(self, ac=None):
        # TODO: Needs to be tested
        if ac is not None:
            ac = np.array(ac)
            assert len(ac) == len(self.active_motor_ids)
            # import pdb
            # pdb.set_trace()
            des_positions = np.clip(
                ac[self.pos_mask],
                self.pos_lims_low[self.active_motor_mask][self.pos_mask],
                self.pos_lims_high[self.active_motor_mask][self.pos_mask],
            )

            des_torques = np.clip(
                ac[self.tor_mask],
                self.tor_lims_low[self.active_motor_mask][self.tor_mask],
                self.tor_lims_high[self.active_motor_mask][self.tor_mask],
            )
            if len(self.active_pos_ids):
                self.motors.set_des_pos(self.active_pos_ids, des_positions)
            if len(self.active_tor_ids):
                self.motors.set_des_torque(self.active_tor_ids, des_torques)

        return self.get_obs(), 0.0, False, {}

    def close(self):
        if self.sensing:
            self.sensing.close()
        if not self._dxl_connected:
            return

        if self.motors:
            if not self._close_engaged:
                self.motors.close(self.motor_ids)
            else:
                shutdown_torques = {key: 0.0 for key in self.torque_control_ids}
                try:
                    for mid, t in self.config.dynamixel.shutdown_torques.items():
                        if mid in self.torque_control_ids:
                            shutdown_torques[mid] = t

                    self.motors.setIndividual_des_torque(
                        self.torque_control_ids, shutdown_torques
                    )
                except KeyError:
                    pass

                self.motors.close([])

            self._dxl_connected = False
