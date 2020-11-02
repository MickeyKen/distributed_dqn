#!/usr/bin/python
import matplotlib.pyplot as plt

path = 'Alien-v0_output.txt'
isServiceCount = True

if __name__ == '__main__':

    actor_num = 2
    a_1 = []
    a_2 = []


    xp = []
    yp = []
    yp2 = []
    average_xp = []
    average_yp = []
    average_yp2 = []

    ave_num = 100

    flag = 0
    sum1 = 0
    sum2 = 0

    count = 0

    fig = plt.figure()

    plt.ion()
    # plt.title('Simple Curve Graph')
    plt.xlabel('Episode')
    plt.ylabel('Reward')
    # plt.xlim(0,1500)
    plt.ylim(-300,150)
    plt.grid()
    with open(path) as f:
        xp.append(0)
        yp.append(0)
        for s_line in f:
            if str(s_line.split(' ')[0]) == "EPISODE:":
                print (s_line.split(' '))
                for i in range(len(s_line.split(' '))):
                    if s_line.split(' ')[i] == "TOTAL_REWARD:":
                        # print (s_line.split(' ')[i], s_line.split(' ')[i+1], s_line.split(' ')[i+2])
                        if s_line.split(' ')[i+1] == "":
                            if s_line.split(' ')[i+2] == "":
                                # print (s_line.split(' '))
                                num1 = int(s_line.split(' ')[i+3])
                            else:
                                # print (s_line.split(' ')[i+2])
                                num1 = int(s_line.split(' ')[i+2])
                        else:
                            # print (s_line.split(' ')[i+1])
                            num1 = int(s_line.split(' ')[i+1])

                xp.append(count + 1)
                yp.append(num1)
                num10 = count // ave_num

                if num10 == flag:
                    sum1 += float(num1)
                else:
                    sum1 = sum1 / float(ave_num)
                    average_xp.append((flag+1)*ave_num)
                    average_yp.append(sum1)
                    sum1 = 0
                    sum1 += float(num1)
                    flag += 1
                count += 1
                # print (count)
        # plt.plot(xp,yp, color="#a9ceec", alpha=0.5)
        plt.plot(average_xp,average_yp, color="#00529a")
        plt.draw()
        fig.savefig("result_multi_reward.png")
        plt.pause(0)
