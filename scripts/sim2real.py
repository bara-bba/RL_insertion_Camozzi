import tensorflow as tf
from garage.experiment import Snapshotter

import rtde_control
import rtde_receive

from UR5_env import UR5

param_dir = "/home/bara/PycharmProjects/Garage/data/local/experiment/garage_sac_panda_position/"
snapshotter = Snapshotter()

with tf.compat.v1.Session() as sess:
    print("extrating parameters from file %s ..." % param_dir)
    data = snapshotter.load(param_dir)

policy = data['algo'].policy
env = UR5()

from garage import rollout

path = rollout(env, policy)
print(path)

# from garage.experiment import Snapshotter
# snapshotter = Snapshotter()
# snapshot = snapshotter.load('path/to/snapshot/dir')
#
# expert = snapshot['algo'].policy
# env = snapshot['env']  # We assume env is the same
#
# # Setup new experiment
# from garage import wrap_experiment
# from garage.sampler import LocalSampler
# from garage.torch.algos import BC
# from garage.torch.policies import GaussianMLPPolicy
# from garage.trainer import Trainer
#
# @wrap_experiment
# def bc_with_pretrained_expert(ctxt=None):
#     trainer = Trainer(ctxt)
#     policy = GaussianMLPPolicy(env.spec, [8, 8])
#     batch_size = 1000
#     sampler = LocalSampler(agents=expert,
#                            envs=env,
#                            max_episode_length=env.spec.max_episode_length)
#     algo = BC(env.spec,
#               policy,
#               batch_size=batch_size,
#               source=expert,
#               sampler=sampler,
#               policy_lr=1e-2,
#               loss='log_prob')
#     trainer.setup(algo, env)
#     trainer.train(100, batch_size=batch_size)
#
#
# bc_with_pretrained_expert()