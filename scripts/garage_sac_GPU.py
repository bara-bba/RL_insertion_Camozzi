#!/usr/bin/env python3
"""This is an example to train a task with SAC algorithm written in PyTorch."""
import os
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

from garage import wrap_experiment
from garage.envs import GymEnv, normalize
from garage.experiment import deterministic
from garage.replay_buffer import PathBuffer
from garage.sampler import MultiprocessingSampler, RaySampler, VecWorker, WorkerFactory, LocalSampler, DefaultWorker, FragmentWorker
from garage.trainer import TFTrainer, Trainer
from garage.torch import set_gpu_mode
from garage.torch.algos import SAC_OLD
from garage.torch.policies import TanhGaussianMLPPolicy
from garage.torch.q_functions import ContinuousMLPQFunction

from panda_env_rel import PandaEnv

"""Snapshotter snapshots training data.

When training, it saves data to binary files. When resuming,
it loads from saved data.

Args:
    snapshot_dir (str): Path to save the log and iteration snapshot.
    snapshot_mode (str): Mode to save the snapshot. Can be either "all"
        (all iterations will be saved), "last" (only the last iteration
        will be saved), "gap" (every snapshot_gap iterations are saved),
        "gap_and_last" (save the last iteration as 'params.pkl' and save
        every snapshot_gap iteration separately), "gap_overwrite" (same as
        gap but overwrites the last saved snapshot), or "none" (do not
        save snapshots).
    snapshot_gap (int): Gap between snapshot iterations. Wait this number
        of iterations before taking another snapshot.

"""

from gym.utils.env_checker import check_env

@wrap_experiment(snapshot_mode='last')
def garage_sac_panda_position(ctxt=None, seed=1):
    """Set up environment and algorithm and run the task.

    Args:
        ctxt (garage.experiment.ExperimentContext): The experiment
            configuration used by Trainer to create the snapshotter.
        seed (int): Used to seed the random number generator to produce
            determinism.

    """
    deterministic.set_seed(seed)

    trainer = Trainer(snapshot_config=ctxt)

    env = normalize(GymEnv(PandaEnv(), max_episode_length=2000), normalize_obs=True, normalize_reward=True)
    #
    # env = DMControlEnv.from_suite(PandaEnv, )

    # env = GymEnv(PandaEnv(),max_episode_length=1000)

    policy = TanhGaussianMLPPolicy(
        env_spec=env.spec,
        hidden_sizes=[256, 256],
        hidden_nonlinearity=nn.ReLU,
        output_nonlinearity=None,
        min_std=np.exp(-20.),
        max_std=np.exp(2.),
    )

    qf1 = ContinuousMLPQFunction(env_spec=env.spec,
                                 hidden_sizes=[256, 256],
                                 hidden_nonlinearity=F.relu)

    qf2 = ContinuousMLPQFunction(env_spec=env.spec,
                                 hidden_sizes=[256, 256],
                                 hidden_nonlinearity=F.relu)

    replay_buffer = PathBuffer(capacity_in_transitions=int(1e6))

    # worker_factory = WorkerFactory(max_episode_length=env.spec.max_episode_length,
    #                                is_tf_worker=True,
    #                                n_workers=1,
    #                                worker_class=DefaultWorker
    #                                )

    sampler = LocalSampler(max_episode_length=env.spec.max_episode_length,
                           agents=policy,
                           envs=env,
                           worker_class=FragmentWorker
    )

    sac = SAC_OLD(env_spec=env.spec,
                  policy=policy,
                  qf1=qf1,
                  qf2=qf2,
                  sampler=sampler,
                  gradient_steps_per_itr=1000,
                  max_episode_length_eval=5000,
                  replay_buffer=replay_buffer,
                  min_buffer_size=1e4,
                  target_update_tau=5e-3,
                  discount=0.99,
                  buffer_batch_size=256,
                  reward_scale=1.,
                  steps_per_epoch=1)

    if torch.cuda.is_available():
        set_gpu_mode(True)
    else:
        set_gpu_mode(False)
    sac.to()
    trainer.setup(algo=sac, env=env)
    # trainer.train(n_epochs=3000, batch_size=3000, plot=True)
    trainer.train(n_epochs=1000, batch_size=1000)


s = np.random.randint(0, 1000)
garage_sac_panda_position(seed=521)