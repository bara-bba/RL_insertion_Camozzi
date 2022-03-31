import tensorflow as tf
import cloudpickle

with tf.compat.v1.Session() as sess:
    with open("/home/bara/PycharmProjects/Garage/data/local/experiment/garage_td3_panda_2/params.pkl", "rb") as pickle_file:
        data = cloudpickle.load(pickle_file)
        policy = data['algo'].policy
        print(type(policy))
        # env = data['env']
        # while True:
        #     path = rollout(env,
        #                    policy,
        #                    max_episode_length=args.max_episode_length,
        #                    animated=True)
        #     if not query_yes_no('Continue simulation?'):
        #         break