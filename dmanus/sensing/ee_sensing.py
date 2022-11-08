from typing import Dict, Tuple

import omegaconf
import yaml  # type: ignore

from franka_demo.utils.constants import ObsDict


class EESensing:
    def __init__(self, sensing_cfg: omegaconf.DictConfig):
        self.config = sensing_cfg

        self.connected = False

    def reset(self) -> Tuple[ObsDict, Dict]:
        return self.get_obs(), {}

    def get_obs(self) -> Dict[str, ObsDict]:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError
