#! /usr/bin/env python
import os
import multiprocessing as mp

import rospy
import math
import time
import tf

import numpy as np

# import deepq_pararell

from std_msgs.msg import Bool
from std_msgs.msg import Float64, Int16
from sensor_msgs.msg import LaserScan

from si2020_msgs.msg import Experiment
from si2020_msgs.srv import *

# import tensorflow as tf
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()
from keras.models import *
from keras.layers import *
from keras.utils import plot_model
from keras import backend as K

from environment import Env1

#adjust as much as you need, limited number of physical cores of your cpu
numberOfCpuCore_to_be_used = 2

continue_execution = False


if not continue_execution:
    #Each time we take a sample and update our weights it is called a mini-batch.
    #Each time we run through the entire dataset, it's called an epoch.
    #PARAMETER LIST
    epochs = 3000000
    steps = 150
    updateTargetNetwork = 10000
    explorationRate = 1
    minibatch_size = 64
    learnStart = 64
    learningRate = 5e-3
    discountFactor = 0.99
    memorySize = 1000000
    network_inputs = 7
    network_outputs = 8

    ### number of hiddenLayer ###
    network_structure = [56,28]
    current_epoch = 0

    # deepQ = deepq.DeepQ(network_inputs, network_outputs, memorySize, discountFactor, learningRate, learnStart)
    # deepQ.initNetworks(network_structure)

# -- constants of Game
NUM_STATES = network_inputs
NUM_ACTIONS = network_outputs
NONE_STATE = np.zeros(NUM_STATES)

# -- constants of LocalBrain
MIN_BATCH = 5
LOSS_V = .5  # v loss coefficient
LOSS_ENTROPY = .01  # entropy coefficient
LEARNING_RATE = learningRate
RMSPropDecaly = 0.99

GAMMA = discountFactor
N_STEP_RETURN = 5
GAMMA_N = GAMMA ** N_STEP_RETURN

N_WORKERS = 2
Tmax = 200

EPSILON_START = explorationRate
EPSILON_END = 0.05
EPSILON_STEPS = 200*N_WORKERS

class ParameterServer:
    def __init__(self):
        with tf.variable_scope("parameter_server"):
            self.model = self._build_model()

        self.weights_params = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope="parameter_server")
        self.optimizer = tf.train.RMSPropOptimizer(LEARNING_RATE, RMSPropDecaly)

    def _build_model(self):
        l_input = Input(batch_shape=(None, NUM_STATES))
        l_dense = Dense(16, activation='relu')(l_input)
        out_actions = Dense(NUM_ACTIONS, activation='softmax')(l_dense)
        out_value = Dense(1, activation='linear')(l_dense)
        model = Model(inputs=[l_input], outputs=[out_actions, out_value])
        plot_model(model, to_file='A3C.png', show_shapes=True)
        return model

class LocalBrain:
    def __init__(self, name, parameter_server):
        with tf.name_scope(name):
            self.train_queue = [[], [], [], [], []]
            K.set_session(SESS)
            self.model = self._build_model()
            self._build_graph(name, parameter_server)

    def _build_model(self):
        l_input = Input(batch_shape=(None, NUM_STATES))
        l_dense = Dense(16, activation='relu')(l_input)
        out_actions = Dense(NUM_ACTIONS, activation='softmax')(l_dense)
        out_value = Dense(1, activation='linear')(l_dense)
        model = Model(inputs=[l_input], outputs=[out_actions, out_value])
        model._make_predict_function()  # have to initialize before threading
        return model

    def _build_graph(self, name, parameter_server):
        self.s_t = tf.placeholder(tf.float32, shape=(None, NUM_STATES))
        self.a_t = tf.placeholder(tf.float32, shape=(None, NUM_ACTIONS))
        self.r_t = tf.placeholder(tf.float32, shape=(None, 1))  # not immediate, but discounted n step reward

        p, v = self.model(self.s_t)

        log_prob = tf.log(tf.reduce_sum(p * self.a_t, axis=1, keep_dims=True) + 1e-10)
        advantage = self.r_t - v
        loss_policy = - log_prob * tf.stop_gradient(advantage)
        loss_value = LOSS_V * tf.square(advantage)  # minimize value error
        entropy = LOSS_ENTROPY * tf.reduce_sum(p * tf.log(p + 1e-10), axis=1, keep_dims=True)  # maximize entropy (regularization)
        self.loss_total = tf.reduce_mean(loss_policy + loss_value + entropy)

        self.weights_params = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope=name)

        self.grads = tf.gradients(self.loss_total, self.weights_params)

        self.update_global_weight_params = \
            parameter_server.optimizer.apply_gradients(zip(self.grads, parameter_server.weights_params))

        self.pull_global_weight_params = [l_p.assign(g_p)
                                          for l_p, g_p in zip(self.weights_params, parameter_server.weights_params)]

        self.push_local_weight_params = [g_p.assign(l_p)
                                          for g_p, l_p in zip(parameter_server.weights_params, self.weights_params)]

    def pull_parameter_server(self):
        SESS.run(self.pull_global_weight_params)

    def push_parameter_server(self):
        SESS.run(self.push_local_weight_params)

    def update_parameter_server(self):
        if len(self.train_queue[0]) < MIN_BATCH:
            return

        s, a, r, s_, s_mask = self.train_queue
        self.train_queue = [[], [], [], [], []]
        s = np.vstack(s)
        a = np.vstack(a)
        r = np.vstack(r)
        s_ = np.vstack(s_)
        s_mask = np.vstack(s_mask)

        _, v = self.model.predict(s_)


        r = r + GAMMA_N * v * s_mask
        feed_dict = {self.s_t: s, self.a_t: a, self.r_t: r}
        SESS.run(self.update_global_weight_params, feed_dict)

    def predict_p(self, s):
        p, v = self.model.predict(s)
        return p

    def train_push(self, s, a, r, s_):
        self.train_queue[0].append(s)
        self.train_queue[1].append(a)
        self.train_queue[2].append(r)

        if s_ is None:
            self.train_queue[3].append(NONE_STATE)
            self.train_queue[4].append(0.)
        else:
            self.train_queue[3].append(s_)
            self.train_queue[4].append(1.)


class Agent:
    def __init__(self, name, parameter_server):
        self.brain = LocalBrain(name, parameter_server)
        self.memory = []
        self.R = 0.

    def act(self, s):
        if frames >= EPSILON_STEPS:
            epsilon = EPSILON_END
        else:
            epsilon = EPSILON_START + frames * (EPSILON_END - EPSILON_START) / EPSILON_STEPS  # linearly interpolate

        if random.random() < epsilon:
            return random.randint(0, NUM_ACTIONS - 1)
        else:
            s = np.array([s])
            p = self.brain.predict_p(s)

            # a = np.argmax(p)

            a = np.random.choice(NUM_ACTIONS, p=p[0])
            return a

    def advantage_push_local_brain(self, s, a, r, s_):
        def get_sample(memory, n):
            s, a, _, _ = memory[0]
            _, _, _, s_ = memory[n - 1]
            return s, a, self.R, s_

        a_cats = np.zeros(NUM_ACTIONS)
        a_cats[a] = 1
        self.memory.append((s, a_cats, r, s_))

        self.R = (self.R + r * GAMMA_N) / GAMMA

        if s_ is None:
            while len(self.memory) > 0:
                n = len(self.memory)
                s, a, r, s_ = get_sample(self.memory, n)
                self.brain.train_push(s, a, r, s_)
                self.R = (self.R - self.memory[0][2]) / GAMMA
                self.memory.pop(0)

            self.R = 0

        if len(self.memory) >= N_STEP_RETURN:
            s, a, r, s_ = get_sample(self.memory, N_STEP_RETURN)
            self.brain.train_push(s, a, r, s_)
            self.R = self.R - self.memory[0][2]
            self.memory.pop(0)

class Environment:
    total_reward_vec = np.zeros(10)
    count_trial_each_thread = 0

    def __init__(self, name, thread_type, parameter_server):
        self.name = name
        self.port = (name*11) + 50
        self.ROS_MASTER_URI = "113" + str(self.port)
        self.thread_type = thread_type
        is_training = True
        self.env = Env1(is_training, self.ROS_MASTER_URI)
        self.agent = Agent(name, parameter_server)


    def run(self):
        os.environ['ROS_MASTER_URI'] = "http://localhost:" + self.ROS_MASTER_URI + '/'
        rospy.init_node('pararell_agent_' + self.ROS_MASTER_URI)
        self.agent.brain.pull_parameter_server()
        global frames
        global isLearned

        s = self.env.reset()
        R = 0
        step = 0
        while True:

            a = self.agent.act(s)
            s_, r, done, arrive, reach = self.env.step(a, last_a)
            last_a = a
            self.agent.advantage_push_local_brain(s, a, r, s_)
            s = s_
            R += r
            if r == 150:
                done = True

            step += 1
            frames += 1

            if done or step == 150:
                done = True
                if not(isLearned) and self.thread_type is 'learning':
                    self.agent.brain.update_parameter_server()
                    self.agent.brain.pull_parameter_server()

            if done:
                self.total_reward_vec = np.hstack((self.total_reward_vec[1:], step))
                self.count_trial_each_thread += 1
                break

        print("Thread name: "+ ROS_MASTER_URI + ",episode: "+str(self.count_trial_each_thread) + ",step:" + str(step)+",average step: "+str(self.total_reward_vec.mean()))

        if self.total_reward_vec.mean() > 120:
            isLearned = True
            time.sleep(2.0)
            self.agent.brain.push_parameter_server()

class Worker_thread:
    def __init__(self, thread_name, thread_type, parameter_server):
        self.environment = Environment(thread_name, thread_type, parameter_server)
        self.thread_type = thread_type

    def run(self):
        # while True:
        if not(isLearned) and self.thread_type is 'learning':
            self.environment.run()

        if not(isLearned) and self.thread_type is 'test':
            time.sleep(1.0)

        if isLearned and self.thread_type is 'learning':
            time.sleep(3.0)

        if isLearned and self.thread_type is 'test':
            time.sleep(3.0)
            self.environment.run()

frames = 0
isLearned = False
SESS = tf.Session()

with tf.device("/cpu:0"):
    parameter_server = ParameterServer()
    threads = []

    for i in range(N_WORKERS):
        thread_name = int(i)
        threads.append(Worker_thread(thread_name=thread_name, thread_type="learning", parameter_server=parameter_server))

    # threads.append(Worker_thread(thread_name="test_thread", thread_type="test", parameter_server=parameter_server))

COORD = tf.train.Coordinator()
SESS.run(tf.global_variables_initializer())

running_threads = []
for worker in threads:
    job = lambda: worker.run()
    t = threading.Thread(target=job)
    t.start()
