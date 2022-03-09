import numpy as np
import csv as csv

from gym import utils
from gym.envs.mujoco import mujoco_env

# PATH_CSV = '/home/bara/doc/scripts/get_info_csv/dist.csv'
# file = open(PATH_CSV, 'w', encoding='UTF-8', newline='')
# header = ['step', 'state', 'action']
#
# writer = csv.writer(file)
# writer.writerow(header)

class PandaEnv(mujoco_env.MujocoEnv, utils.EzPickle):

    def __init__(self):
        utils.EzPickle.__init__(self)
        mujoco_env.MujocoEnv.__init__(self, "/home/bara/PycharmProjects/Garage/panda/insert_base.xml", 2)

    def step(self, action):
        counter = 0

        # STEP
        # print("STATE" + str(self.data.qpos.astype(np.float32)))
        # print("ACTION" + str(action))

        # action = self.sim.data.qpos + old_action
        # print("OLD " + str(old_action))
        # print("NEW " + str(action))

        self.do_simulation(action, self.frame_skip)
        counter += 1

        # DISTANCE
        # xpos_insert = self.get_site_xpos("insert_site")
        # xpos_base = self.get_site_xpos("base_site")

        self.diff_vector = self.get_site_xpos("insert_site") - self.get_site_xpos("base_site")

        dist = np.linalg.norm(self.diff_vector)*100                 # Centimeters

        # REWARD
        if dist < 0.3:                                              # Millimiters
            reward_pos = 20
            done = True
        else:
            reward_pos = 0
            done = False

        reward_dist = -dist
        reward_action = -counter/20
        reward = 1.5*reward_dist + reward_pos + reward_action       # More contributions to rewards may be added

        ob = self._get_obs()

        return ob, reward, done, dict(reward_pos=reward_pos, reward_action=reward_action, reward_dist=reward_dist, reward=reward)

    def viewer_setup(self):
        self.viewer.cam.distance = self.model.stat.extent * 0.5

    def reset_model(self):
        c = 0.01
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
