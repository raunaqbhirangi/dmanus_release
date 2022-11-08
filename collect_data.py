import os
import hydra

import omegaconf
import pickle
from datetime import datetime
import numpy as np

from dmanus import DManus
from dmanus.utils import Rate
from policies import StaticDManusPolicyContinuousNoisy


def save_data(expt_dir, prefix, data):

    if not os.path.isdir(expt_dir):
        os.mkdir(expt_dir)

    file_path = os.path.join(expt_dir, prefix + "_data.pkl")
    with open(file_path, "wb") as f:
        pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)


@hydra.main(config_path="conf", config_name="config", version_base="1.2")
def main(cfg: omegaconf.DictConfig):

    main_dir = cfg.policy.data_dir
    # initialize dmanus and policy objects

    dm = DManus(cfg.dmanus.ee_cfg)

    pol = StaticDManusPolicyContinuousNoisy(cfg.policy)
    main_dir = pol.data_dir

    # setup data directory stuff
    if not os.path.isdir(main_dir):
        os.makedirs(main_dir)

    expt_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    expt_dir = os.path.join(main_dir, expt_name)

    rate = Rate(frequency=pol.frequency)

    batch = 0
    while True:
        cmd = input("Enter Command: ")
        if cmd == "s":

            obs_list = []
            reset_obs, reset_info = dm.reset()
            obs_list.append(reset_obs)

            object_type = input("Enter object/material: ")

            # Policy execution
            print("Executing policy...")
            for _ in range(pol.len):
                act = pol()

                obs, _, _, _ = dm.step(act)

                obs_list.append(obs)
                rate.sleep()

            # Squish the dictionary
            out_dict = {k: [] for k in list(obs_list[0].keys())}
            for obs in obs_list:
                for k in out_dict.keys():
                    out_dict[k].append(obs[k])
            out_dict = {k: np.array(v) for k, v in out_dict.items()}
            out_dict.update(reset_info)

            print("Stop data collection\n")
            
            dm.reset()

            save_check = input("Save data [y/n]: ")
            if save_check == "y":
                print("Saving data...")
                save_data(expt_dir, prefix=f"{batch}_{object_type}", data=out_dict)
                batch += 1
        elif cmd == "r":
            dm.reset()
        else:
            break


if __name__ == "__main__":
    main()
