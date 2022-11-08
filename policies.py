import numpy as np
import omegaconf


class DManusPolicy:
    def __init__(self, policy_cfg: omegaconf.DictConfig):

        self.policy_cfg = policy_cfg
        # self.classes = policy_cfg.classes
        self.data_dir = policy_cfg.data_dir
        self.len = policy_cfg.len
        self.iter = 0
        self.iterations = policy_cfg.iterations
        self.frequency = policy_cfg.frequency

    def __call__(self, obs=None):
        raise NotImplementedError


class StaticDManusPolicyDiscrete(DManusPolicy):
    def __init__(self, policy_cfg: omegaconf.DictConfig):

        super().__init__(policy_cfg)
        self.actions = self.base_actions = policy_cfg.actions

        self.act_counter = 0
        self.calls_per_iter = self.len // (self.iterations * len(self.actions))

    def __call__(self, obs=None):
        return_act = self.actions[self.act_counter]
        self.iter += 1
        if self.iter % (self.calls_per_iter) == 0:
            self.act_counter = (self.act_counter + 1) % len(self.actions)
        return return_act


class StaticDManusPolicyContinuous(DManusPolicy):
    def __init__(self, policy_cfg):

        super().__init__(policy_cfg)

        self.current_actions = self.default_pos = policy_cfg.default_pos
        self.base_actions = policy_cfg.actions
        self.calls_per_iter = self.len // (self.iterations * len(self.base_actions))
        self.actions = [[self.base_actions[0]]]

        for aid, action in enumerate(self.base_actions):
            interp_actions = np.linspace(
                action,
                self.base_actions[(aid + 1) % len(self.base_actions)],
                self.calls_per_iter,
            )
            self.actions.append(interp_actions)

        self.actions = np.concatenate((self.actions), axis=0)
        self.act_counter = 0

        self.iter = 0  # how many times you have called policy
        self.closed = -1  # finished closing hand

    def __call__(self, obs=None):
        return_act = self.actions[self.act_counter]
        self.act_counter = (self.act_counter + 1) % len(self.actions)
        self.iter += 1
        return return_act


class StaticDManusPolicyContinuousNoisy(DManusPolicy):
    def __init__(self, policy_cfg):

        super().__init__(policy_cfg)

        self.current_actions = self.default_pos = policy_cfg.default_pos
        self.actions = policy_cfg.actions
        self.calls_per_iter = self.len // (self.iterations * len(self.actions))
        self.interp_actions = [[self.actions[0]]]

        self.noise_std = policy_cfg.noise.std
        self.noise_motors = policy_cfg.noise.motors
        self.active_motors = policy_cfg.active_motors
        self.noise_motors_mask = [id in self.noise_motors for id in self.active_motors]

        for aid, action in enumerate(self.actions):
            interp_actions = np.linspace(
                action, self.actions[(aid + 1) % len(self.actions)], self.calls_per_iter
            )
            self.interp_actions.append(interp_actions)

        self.interp_actions = np.concatenate((self.interp_actions), axis=0)
        self.act_counter = 0

        self.iter = 0  # how many times you have called policy
        self.closed = -1  # finished closing hand

        self.num_pivs = policy_cfg.noise.num_pivots

        self.noise_traj = []
        noise_points = (
            np.random.normal(
                loc=0,
                scale=self.noise_std,
                size=(self.num_pivs + 1, len(self.active_motors)),
            )
            * self.noise_motors_mask
        )

        for id, point in enumerate(noise_points[:-1]):
            interp_noise = np.linspace(
                point,
                noise_points[id + 1],
                self.len // (self.num_pivs - 1),
            )
            self.noise_traj.append(interp_noise)
        # import pdb

        # pdb.set_trace()
        self.noise_traj.append(noise_points[-1])
        self.noise_traj = np.vstack(self.noise_traj)

    def __call__(self, obs=None):

        return_act = (
            self.interp_actions[self.act_counter]
            + self.noise_traj[self.iter % len(self.noise_traj)]
        )
        self.act_counter = (self.act_counter + 1) % len(self.interp_actions)
        self.iter += 1
        return return_act
