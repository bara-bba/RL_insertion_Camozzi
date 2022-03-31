import gym
import numpy as np
import csv as csv

from gym import utils
from gym.envs.mujoco import mujoco_env

# TRY MUJOCO MOVE TILL REACHED, THEN STEP

class PandaEnv(mujoco_env.MujocoEnv, utils.EzPickle):

    def __init__(self):
        self.counter = 0
        utils.EzPickle.__init__(self)
        mujoco_env.MujocoEnv.__init__(self, "/home/bara/PycharmProjects/Garage/panda/insert_base.xml", 1)

    def step(self, action):

        # action = self.sim.data.qpos + old_action

        # print("QPOS: " + str(self.sim.data.qpos))
        # print("OLD " + str(old_action))
        # print("NEW " + str(action))

        self.do_simulation(action, self.frame_skip)

        # DISTANCE
        # xpos_insert = self.get_site_xpos("insert_site")
        # xpos_base = self.get_site_xpos("base_site")

        self.diff_vector = self.get_site_xpos("insert_site") - self.get_site_xpos("base_site")

        dist = np.linalg.norm(self.diff_vector) * 10
        # print( "DIST: " + str(dist))

        # REWARD
        if dist < 0.05:  # Millimiters
            reward_pos = 100
            self.counter = 0
            done = True
        else:
            reward_pos = 0
            self.counter += 1
            done = False

        # print("C: " + str(self.counter))

        reward_dist = -10*dist
        # reward_action = -self.counter/100
        # reward = 10 + reward_dist + reward_pos + 0.1666*reward_action  # More contributions to rewards may be added
        reward = 10 + reward_dist + reward_pos                           # More contributions to rewards may be added

        # print("REWARD_ACTION: " + str(reward_action))
        # print("REWARD_DIST: " + str(reward_dist))
        # print("REWARD_POS: " + str(reward_pos))
        # print("REWARD: " + str(reward))

        ob = self._get_obs()

        return ob, reward, done, dict(reward_pos=reward_pos, reward=reward)
        # return ob, reward, done, dict(reward_pos=reward_pos, reward_action=reward_action, reward_dist=reward_dist, reward=reward)

    def viewer_setup(self):
        self.viewer.cam.distance = self.model.stat.extent * 0.5

    def reset_model(self):
        c = 0.01
        self.counter = 0
        self.set_state(
            self.np_random.uniform(low=-c, high=c, size=self.model.nq),
            np.zeros(self.model.nv)
        )
        return self._get_obs()

    def _get_obs(self):
        return np.concatenate(
            [
                self.sim.data.qpos.flat,
                self.sim.data.qvel.flat,
                self.sim.data.sensordata.flat,
                self.diff_vector.flat,
            ]
        ).astype(np.float32).flatten()
