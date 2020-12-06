#!/usr/bin/python
import matplotlib.pyplot as plt

path = 'param_UD-v6_output.txt'
isServiceCount = True

ACTOR_NUM = 2
AVERAGE_NUM = 100

if __name__ == '__main__':

    rewards = [[0] for j in range(ACTOR_NUM)]
    average_rewards = [[0] for j in range(ACTOR_NUM)]

    rewards_sum = [[0] for j in range(ACTOR_NUM)]

    eps = [[0] for j in range(ACTOR_NUM)]
    average_eps = [[0] for j in range(ACTOR_NUM)]

    epsilons = [[0] for j in range(ACTOR_NUM)]

    flag = 0

    count = 0

    fig = plt.figure()

    plt.ion()
    # plt.title('Simple Curve Graph')
    plt.xlabel('Episode')
    plt.ylabel('Reward')
    # plt.xlim(0,1500)
    plt.ylim(-300,150)
    plt.grid()

    cycle = plt.rcParams['axes.prop_cycle'].by_key()['color']

    with open(path) as f:

        for s_line in f:
            eps_num = int(s_line.split(',')[0])
            actor_num = int(s_line.split(',')[1])
            reward = int(s_line.split(',')[5])
            epsilon = float(s_line.split(',')[4])
            # print eps_num,actor_num, reward

            rewards[actor_num].append(reward)
            eps[actor_num].append(eps_num)
            epsilons[actor_num].append(epsilon)

            if eps_num % 100 == 0 and eps_num > 0:
                average_rewards[actor_num].append(sum(rewards_sum[actor_num]) / len(rewards_sum[actor_num]))
                average_eps[actor_num].append(eps_num)
                rewards_sum = [[0] for j in range(ACTOR_NUM)]
            else:
                rewards_sum[actor_num].append(reward)

        for i in range(ACTOR_NUM):
            label = "epsilon = "+str(epsilons[0][i])
            print label
            plt.plot(average_eps[i],average_rewards[i], color=cycle[i], label=label)

        plt.legend( loc='upper left', borderaxespad=1)
        plt.draw()
        fig.savefig("result_multi_reward.png")
        plt.pause(0)
