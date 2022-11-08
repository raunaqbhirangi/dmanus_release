import time
from collections import OrderedDict

import numpy as np
from omegaconf import DictConfig, OmegaConf
from reskin_sensor import ReSkinProcess

from franka_demo.endeffector.sensing.ee_sensing import EESensing


class EEReskin(EESensing):
    def __init__(self, sensing_cfg: DictConfig):
        super().__init__(sensing_cfg)
        self.config = sensing_cfg
        self.sensor_proc = ReSkinProcess(**sensing_cfg.connection_params)
        self.sensor_proc.start()
        self.connected = True
        self.baseline = None

        try:
            self.baseline_samples = sensing_cfg["num_baseline_samples"]
        except KeyError:
            self.baseline_samples = 100

        # TODO: Fix reskin_sensor to block when starting process
        time.sleep(1.0)
        # self.reset()

    def reset(self):
        # Record baseline and store in default observation
        obs = self.get_obs()

        self._update_baseline()
        info = {"ee_sensing_baseline": self.baseline}

        return obs, info

    def _update_baseline(self):
        raw_data = self.sensor_proc.get_data(self.baseline_samples)
        print("baseline_len: ", len(raw_data))
        _, _, reskin_data, _ = zip(*raw_data)
        self.baseline = np.array(reskin_data)

    def get_obs(self):
        rs_data = self.sensor_proc.get_data(1)
        obs = OrderedDict()
        obs["ee_sensing"] = np.array(rs_data[0].data)
        return obs

    def close(self):
        if self.connected:
            self.sensor_proc.join()
            self.connected = False
