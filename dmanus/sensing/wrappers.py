from typing import Any, Dict, Tuple, Type
import numpy as np

from ee_sensing import EESensing

from scipy.signal import butter
from utils import ObsDict


class EESensingWrapper:
    def __init__(self, ee_sensor: EESensing):
        self.ee_sensor = ee_sensor

    def setup(self) -> None:
        self.ee_sensor.setup()

    def reset(self) -> Tuple[np.ndarray, Dict[str, Any]]:
        return self.ee_sensor.reset()

    def get_obs(self) -> Tuple[np.ndarray, Dict[str, Any]]:
        return self.ee_sensor.get_obs()

    def _get_obs_dict(self) -> ObsDict:
        return self.ee_sensor._get_obs_dict()

    def close(self) -> None:
        self.ee_sensor.close()


class LowpassFilter(EESensingWrapper):
    def __init__(self, ee_sensor):
        super().__init__(ee_sensor)

    def reset(self):
        # TODO: Add functionality here that uses sosfilt_zi to
        # compute initial conditions using the baseline. This will be
        # used in apply_filter when its called the first time and value
        # will be maintained
        pass

    def _get_obs_dict(self):
        obs_dict = super()._get_obs_dict()
        filtered_output = self._apply_filter(obs_dict["ee_sensing"])
        obs_dict["ee_sensing_filtered"] = filtered_output
        return obs_dict

    def _apply_filter(self, obs):
        pass
