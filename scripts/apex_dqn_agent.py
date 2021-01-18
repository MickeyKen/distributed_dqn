#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import multiprocessing as mp
import tensorflow as tf
import numpy as np
import argparse
import time
import gym
import random

import rospy

from actor import Actor
from learner import Learner


def actor_work(args, queues, num):
    # with tf.device('/cpu:0'):
    port = (int(num)*11) + 50
    ROS_MASTER_URI = "113" + str(port)
    os.environ['ROS_MASTER_URI'] = "http://localhost:" + str(ROS_MASTER_URI) + '/'
    rospy.init_node('pararell_agent_' + ROS_MASTER_URI)
    sess = tf.InteractiveSession()
    actor = Actor(args, queues, num, sess, param_copy_interval=2000, send_size=200)
    actor.run()

def leaner_work(args, queues):
    # with tf.device('/gpu:0'):
    sess = tf.InteractiveSession()
    leaner = Learner(args, queues, sess, batch_size=126)
    leaner.run()

def plot():
    pass

# Train Mode
if __name__ == '__main__':
    # rospy.init_node('apex_dqn_agent')

    parser = argparse.ArgumentParser()
    parser.add_argument('--num_actors', type=int, default=4, help='number of Actors')
    parser.add_argument('--env_name', type=str, default='Alien-v0', help='Environment of Atari2600 games')
    parser.add_argument('--train', type=int, default=1, help='train mode or test mode')
    parser.add_argument('--test_gui', type=int, default=0, help='decide whether you use GUI or not in test mode')
    parser.add_argument('--load', type=int, default=0, help='loading saved network')
    parser.add_argument('--network_path', type=str, default=0, help='used in loading and saving (default: \'saved_networks/<env_name>\')')
    parser.add_argument('--replay_memory_size', type=int, default=2000000, help='replay memory size')
    parser.add_argument('--initial_memory_size', type=int, default=20000, help='Learner waits until replay memory stores this number of transition')
    parser.add_argument('--num_episodes', type=int, default=10000, help='number of episodes each agent plays')
    parser.add_argument('--frame_width', type=int, default=84, help='width of input frames')
    parser.add_argument('--frame_height', type=int, default=84, help='height of input frames')
    parser.add_argument('--state_length', type=int, default=9, help='number of input frames')
    parser.add_argument('--n_step', type=int, default=3, help='n step bootstrap target')
    parser.add_argument('--gamma', type=float, default=0.99, help='discount factor')

    args = parser.parse_args()

    if args.network_path == 0:
        args.network_path = 'saved_networks/' + args.env_name
    if not args.load:
        assert not os.path.exists('saved_networks/'+args.env_name), 'Saved network already exists.'
    if not os.path.exists(args.network_path):
        os.makedirs(args.network_path)

    if args.train:
        assert not os.path.exists(args.env_name+'_output.txt'), 'Output file already exists. Change file name.'

    if args.train:
        transition_queue = mp.Queue(100)

        param_queue = mp.Queue(args.num_actors)
        ps = [mp.Process(target=leaner_work, args=(args, (transition_queue, param_queue)))]

        for i in range(args.num_actors):
            ps.append(mp.Process(target=actor_work, args=(args, (transition_queue, param_queue), i)))

        for p in ps:
            p.start()
            time.sleep(0.5)

        for p in ps:
            p.join()

    # Test Mode
    else:
        from test_agent import Agent
        from environment import Env1
        rospy.init_node('apex_test_agent')
        env = Env1(False, "11311")
        # env = gym.wrappers.Monitor(env, args.network_path, force=True)
        sess = tf.InteractiveSession()
        agent = Agent(args, sess)
        t = 0
        total_reward = 0
        for episode in range(10):
            terminal = False
            observation = env.reset()
            last_action = 0

            # state = agent.get_initial_state(observation, last_observation)
            # while not terminal:
            for i in range(150):
                last_observation = observation
                action = agent.get_action_at_test(observation)
                observation, reward, terminal, _, _ = env.step(action, last_action, i)
                if reward == 150 or terminal:
                    terminal = True
                    break

                total_reward += reward
                t += 1

            text = 'EPISODE: {0:6d} / STEPS: {1:5d} / EPSILON: {2:.5f} / TOTAL_REWARD: {3:3.0f}'.format(
                episode + 1, t, agent.test_epsilon, total_reward)
            print(text)
            total_reward = 0
            t = 0
