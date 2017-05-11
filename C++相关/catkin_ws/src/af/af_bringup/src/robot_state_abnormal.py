#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pyaudio import PyAudio, paInt16
import numpy as np
from datetime import datetime
import math
import wave
import rospy

from af_msgs.msg import RobotState
robot_state = RobotState()

background_eng = 0

Fs = 11026  # 采样频率

pt_step = math.floor(Fs / 2)  # 0.5s的采样点数，求能量点的数据长度

delta_time = 2.5  # 以2.5s为间隔进行平均
avrg_time = 10  # 选取10个能量点求平均能量

# last_time=10# 求10次录音进行监测，求完终止整个程序！！！即最大的录音次数。
# last_time=2  # 10+5=15s录音
last_time = 16  # 10s + 40s =50s录音

alarm_cnt = 0  # 报警计数

NUM_SAMPLES = int(Fs * delta_time)  # pyAudio内部缓存的块的大小，2.5s读取点数
NUM_TEST = 4 * NUM_SAMPLES  # 一次读取4个2.5s，即10s数据的总点数

# 初始化变量为列表， list
save_count = 0
save_buffer = []
eng_step = []  # 0.5s为间隔计算能量
avrg_eng_step = []  # 10个能量点求平均能量
alarm_all = []  # 每个时间点报警状态记录
bkgrd_eng = []  # 背景数据汇总数组记录每个背景点的值

########################################################################
# 先读10s录音，进行初次报警监测
########################################################################
# 开启声音输入 & 读取录音数据
pa = PyAudio()
stream = pa.open(format=paInt16, channels=1, rate=Fs, input=True,
                 frames_per_buffer=NUM_SAMPLES)
string_audio_data = stream.read(NUM_TEST)  # !!!!!! 读取10s录音 !!!!!!
y1 = np.fromstring(string_audio_data, dtype=np.short)  # 读取录音数据：y1

ny = len(y1)  # 10s录音数据长度
N_step = int(math.floor(ny / pt_step))  # 10s录音中的能量点数 (20)

# 求所有能量点 & 求平均能量
# for循环，求10s内所有能量点
for i_step in range(0, N_step):  # 循环从0开始，循环次数为 N_step(20)
    abs_y1_smth = np.abs(y1[i_step * pt_step: (i_step + 1) * pt_step])  # 0.5s内能量取绝对值
    eng_step.append(np.sum(abs_y1_smth))  # 再求和

# for循环，求平均能量(2.5s内平均) (5为2.5/0.5 2.5秒内的能量点数)
for i_avrg in range(0, int(math.floor(N_step / 5))):  # 循环从0开始，循环次数为后面int结果(20/5 = 4)
    # avrg_eng_step.append(np.sum(eng_step[(i_avrg * 5):(i_avrg + 1) * 5]) / avrg_time)
    avrg_eng_step.append(np.sum(eng_step[(i_avrg * 5):(i_avrg + 1) * 5]) / avrg_time)  # ??为什么求平均用10

# 对10s录音的平均能量点进行报警监测
# if判断，根据左右两边点比较确定当前点的报警状态
# if (avrg_eng_step[2]>(avrg_eng_step[0]*5)):# 报警点
if avrg_eng_step[2] > (avrg_eng_step[0] * 2.4):  # 报警点
    alarm_sign = 1
    background_eng = avrg_eng_step[0]

# elseif (avrg_eng_step(3)>(background_eng*2))% possible changing point
# elif (avrg_eng_step[2]>(avrg_eng_step[0]*2)):  # 过渡点
elif avrg_eng_step[2] > (avrg_eng_step[0] * 1.5):  # 过渡点
    alarm_sign = -1
    background_eng = avrg_eng_step[0]

else:  # 背景点
    alarm_sign = 0
    background_eng = avrg_eng_step[1]

# 报警状态记录，报警次数累加
alarm_all.append(alarm_sign)
alarm_cnt += 1

# bkgrd_cnt=1# 记录背景点点数
# bkgrd_cnt = bkgrd_cnt+1# 背景点数累加一次
# background_eng = avrg_eng_step[0]# 背景能量初始化
# bkgrd_eng[bkgrd_cnt] = background_eng# 背景数据汇总数组记录每个背景点的值
bkgrd_eng.append(background_eng)

# ########################################################################
# # 开始while循环，进行后续状态报警监测，每次读取2.5s录音进行实时监测，总计读10次，终止程序
# ########################################################################
# loop_sign = 0  # while循环判断的状态量
#
# # 在循环开始前，输出能量点和平均能亮点
# # print"----- result of 10s record -----"
# # print "****eng_step"
# # print(eng_step)
# # print "****avrg_eng_step"
# # print (avrg_eng_step)
#
# # while True:
# # 读入NUM_SAMPLES个取样
# # string_audio_data = stream.read(NUM_SAMPLES)
# # print(string_audio_data,)
# # # 将读入的数据转换为数组
# # audio_data = np.fromstring(string_audio_data, dtype=np.short)
# # # 计算大于LEVEL的取样的个数
# # print np.max(audio_data)
# # save_buffer.append(string_audio_data)
# # # 如果个数大于COUNT_NUM，则至少保存SAVE_LENGTH个块
# # print(save_buffer)
# # filename = datetime.now().strftime("%Y-%m-%d_%H_%M_%S") + ".wav"
# # save_wave_file(filename, save_buffer)
# # save_buffer = []
# # print filename, "saved"
# print "loooooping"
# # while循环，求平均能量
# while loop_sign == 0:  # 循环在条件成立时执行，否则终止
#
#     n_eng_step = len(eng_step)  # 已计算的能量点序列的当前长度
#     n_avrg_eng_step = len(avrg_eng_step)  # 已计算的平均能量点序列的当前长度
#
#     # 开启录音2.5s & 求新的平均能量点
#     print "读取2.5s"
#     string_audio_data = stream.read(NUM_SAMPLES)  # !!!!!! 读取 2.5s 录音 !!!!!!
#     print "读取结束"
#     y_next = np.fromstring(string_audio_data, dtype=np.short)  # 读取录音数据：y_next
#     ny = len(y_next)  # 新的录音数据长度
#     N_step = int(math.floor(ny / pt_step))  # 新的录音数据包含的能量点数
#
#     # 输出当前录音数据的能量点数和录音长度
#     # print "!!!!"
#     # print N_step
#     # print ny
#     # print "!!!!"
#
#     # for循环，求新的2.5s录音数据的所有能量点
#     # for i_step in range(1,N_step):
#     for i_step in range(0, N_step):  # 循环从0开始，次数为 N_step
#         abs_y1_smth = np.abs(y_next[i_step * pt_step: (i_step + 1) * pt_step])
#         eng_step.append(np.sum(abs_y1_smth))  # 将新计算点加入list的尾部
#
#     # 输出所求新的能量点
#     # print "****eng_step"
#     # print(eng_step)
#
#     # 求新的平均能量点
#     # avrg_eng_step.append(np.sum(eng_step[((n_avrg_eng_step-1)*5):(n_eng_step+4)])/ avrg_time)
#     avrg_eng_step.append(np.sum(eng_step[n_avrg_eng_step * 5:(n_avrg_eng_step + 1) * 5]) / avrg_time)
#
#     print "#######"
#     print avrg_eng_step[-1]
#     print background_eng
#     print "&&&&&&&"
#     # 输出新求的能量点和平均能量点
#     # print "****"
#     # print eng_step[(n_avrg_eng_step * 5):(n_eng_step + 4)]# 新求的能量点
#     # print "****avrg_eng_step"
#     # print (avrg_eng_step)# 新求的平均能量点
#
#     # 判断新点报警状态
#     # if判断，决定新的平均能量点的报警状态
#     if alarm_sign == 0:  # 当前是非报警状态
#         # if (avrg_eng_step[n_avrg_eng_step] > (avrg_eng_step[n_avrg_eng_step - 1] * 2)):
#         # if (avrg_eng_step[n_avrg_eng_step] > (avrg_eng_step[n_avrg_eng_step-2]*5)):# 是报警点
#         # if (avrg_eng_step[n_avrg_eng_step] > (background_eng*5)):# 是报警点
#         if avrg_eng_step[n_avrg_eng_step] > (background_eng * 2):  # 是报警点
#             alarm_sign = 1  # 报警
#
#         # elif (avrg_eng_step[n_avrg_eng_step-1] > (background_eng*2)):# 是过渡点
#         elif avrg_eng_step[n_avrg_eng_step] > (background_eng * 1.5):  # 是过渡点
#             alarm_sign = -1  # 过渡点
#
#         else:  # 是非报警点
#             alarm_sign = 0  # 非报警
#
#             # background_eng = avrg_eng_step[n_avrg_eng_step]
#             background_eng = avrg_eng_step[n_avrg_eng_step]  # 更新背景值
#             bkgrd_eng.append(background_eng)  # 增添新的背景值
#
#     elif alarm_sign == 1:  # 当前是报警状态
#         # if (avrg_eng_step[n_avrg_eng_step] > (background_eng * 2)):
#         # if (avrg_eng_step[n_avrg_eng_step - 1] > (background_eng * 2)):
#         # if (avrg_eng_step[n_avrg_eng_step-1] > (background_eng*5)):# 是报警点
#         if avrg_eng_step[n_avrg_eng_step] > (background_eng * 2):  # 是报警点
#             alarm_sign = 1  # 报警
#
#         # elif (avrg_eng_step[n_avrg_eng_step-1] > (background_eng*2)):  # 是过渡点
#         elif avrg_eng_step[n_avrg_eng_step] > (background_eng * 1.5):  # 是过渡点
#             alarm_sign = -1  # 过渡点
#
#         else:  # 是非报警点
#             alarm_sign = 0  # 非报警
#
#             # background_eng = avrg_eng_step[n_avrg_eng_step]
#             background_eng = avrg_eng_step[n_avrg_eng_step]  # 更新背景值
#             bkgrd_eng.append(background_eng)  # 增添新的背景值
#
#     elif alarm_sign == -1:  # 当前是过渡点状态
#         # if (avrg_eng_step[n_avrg_eng_step-1] > (background_eng*5)):  # 是报警点
#         if avrg_eng_step[n_avrg_eng_step] > (background_eng * 2):  # 是报警点
#             alarm_sign = 1  # 报警
#
#         # elif (avrg_eng_step[n_avrg_eng_step-1] > (background_eng*2)):  # 是过渡点
#         elif avrg_eng_step[n_avrg_eng_step] > (background_eng * 1.5):  # 是过渡点
#             alarm_sign = -1  # 过渡点
#
#         else:  # 是非报警点
#             alarm_sign = 0  # 非报警
#
#             # background_eng = avrg_eng_step[n_avrg_eng_step]
#             background_eng = avrg_eng_step[n_avrg_eng_step]  # 更新背景值
#             bkgrd_eng.append(background_eng)  # 增添新的背景值
#
#             # else:
#             # alarm_sign = 0
#             # background_eng = avrg_eng_step[n_avrg_eng_step - 1]
#
#     # 增添新的报警状态记录 & 报警次数累加
#     alarm_all.append(alarm_sign)
#     alarm_cnt += 1
#
#     if alarm_sign == 1:
#         print "AAAAAAAAAAAAA"
#     # if判断，决定while循环何时终止
#     # if alarm_cnt  > 20:
#     # if alarm_cnt  > 10:
#     # if alarm_cnt > last_time:
#     #    loop_sign = 1
#
# ###################################################################
# # 循环终止后，输出所有报警点的报警状态
# bkgrd_eng_cnt = len(bkgrd_eng)
# alarm_all_cnt = len(alarm_all)
#
# print "****avrg_eng_step"
# print (avrg_eng_step)  # 新求的平均能量点
#
# print "****  alarm_all"
# print(alarm_all)
# print "****  alarm_all_cnt"
# print(alarm_all_cnt)
#
# print "****  bkgrd_eng"
# print(bkgrd_eng)
# print "****  bkgrd_eng_cnt"
# print(bkgrd_eng_cnt)


def robot_state_abnormal():

    rospy.init_node('robot_state_abnormal', anonymous=True)
    pub = rospy.Publisher('robot_state/abnormal_detection', RobotState, queue_size=1)
    rate = rospy.Rate(100) # 10hz

    seq = 0

    background_eng = 0

    Fs = 11026  # 采样频率

    pt_step = math.floor(Fs / 2)  # 0.5s的采样点数，求能量点的数据长度

    delta_time = 2.5  # 以2.5s为间隔进行平均
    avrg_time = 10  # 选取10个能量点求平均能量

    # last_time=10# 求10次录音进行监测，求完终止整个程序！！！即最大的录音次数。
    # last_time=2  # 10+5=15s录音
    last_time = 16  # 10s + 40s =50s录音

    alarm_cnt = 0  # 报警计数

    NUM_SAMPLES = int(Fs * delta_time)  # pyAudio内部缓存的块的大小，2.5s读取点数
    NUM_TEST = 4 * NUM_SAMPLES  # 一次读取4个2.5s，即10s数据的总点数

    # 初始化变量为列表， list
    save_count = 0
    save_buffer = []
    eng_step = []  # 0.5s为间隔计算能量
    avrg_eng_step = []  # 10个能量点求平均能量
    alarm_all = []  # 每个时间点报警状态记录
    bkgrd_eng = []  # 背景数据汇总数组记录每个背景点的值

    ########################################################################
    # 先读10s录音，进行初次报警监测
    ########################################################################
    # 开启声音输入 & 读取录音数据
    pa = PyAudio()
    stream = pa.open(format=paInt16, channels=1, rate=Fs, input=True,
                     frames_per_buffer=NUM_SAMPLES)
    string_audio_data = stream.read(NUM_TEST)  # !!!!!! 读取10s录音 !!!!!!
    y1 = np.fromstring(string_audio_data, dtype=np.short)  # 读取录音数据：y1

    ny = len(y1)  # 10s录音数据长度
    N_step = int(math.floor(ny / pt_step))  # 10s录音中的能量点数 (20)

    # 求所有能量点 & 求平均能量
    # for循环，求10s内所有能量点
    for i_step in range(0, N_step):  # 循环从0开始，循环次数为 N_step(20)
        abs_y1_smth = np.abs(y1[i_step * pt_step: (i_step + 1) * pt_step])  # 0.5s内能量取绝对值
        eng_step.append(np.sum(abs_y1_smth))  # 再求和

    # for循环，求平均能量(2.5s内平均) (5为2.5/0.5 2.5秒内的能量点数)
    for i_avrg in range(0, int(math.floor(N_step / 5))):  # 循环从0开始，循环次数为后面int结果(20/5 = 4)
        # avrg_eng_step.append(np.sum(eng_step[(i_avrg * 5):(i_avrg + 1) * 5]) / avrg_time)
        avrg_eng_step.append(np.sum(eng_step[(i_avrg * 5):(i_avrg + 1) * 5]) / avrg_time)  # ??为什么求平均用10

    # 对10s录音的平均能量点进行报警监测
    # if判断，根据左右两边点比较确定当前点的报警状态
    # if (avrg_eng_step[2]>(avrg_eng_step[0]*5)):# 报警点
    if avrg_eng_step[2] > (avrg_eng_step[0] * 2):  # 报警点
        alarm_sign = 1
        background_eng = avrg_eng_step[0]

    # elseif (avrg_eng_step(3)>(background_eng*2))% possible changing point
    # elif (avrg_eng_step[2]>(avrg_eng_step[0]*2)):  # 过渡点
    elif avrg_eng_step[2] > (avrg_eng_step[0] * 1.5):  # 过渡点
        alarm_sign = -1
        background_eng = avrg_eng_step[0]

    else:  # 背景点
        alarm_sign = 0
        background_eng = avrg_eng_step[1]

    # 报警状态记录，报警次数累加
    alarm_all.append(alarm_sign)
    alarm_cnt += 1

    # bkgrd_cnt=1# 记录背景点点数
    # bkgrd_cnt = bkgrd_cnt+1# 背景点数累加一次
    # background_eng = avrg_eng_step[0]# 背景能量初始化
    # bkgrd_eng[bkgrd_cnt] = background_eng# 背景数据汇总数组记录每个背景点的值
    bkgrd_eng.append(background_eng)

    while not rospy.is_shutdown():

        n_eng_step = len(eng_step)  # 已计算的能量点序列的当前长度
        n_avrg_eng_step = len(avrg_eng_step)  # 已计算的平均能量点序列的当前长度

        # 开启录音2.5s & 求新的平均能量点
        print "读取2.5s"
        string_audio_data = stream.read(NUM_SAMPLES)  # !!!!!! 读取 2.5s 录音 !!!!!!
        print "读取结束"
        y_next = np.fromstring(string_audio_data, dtype=np.short)  # 读取录音数据：y_next
        ny = len(y_next)  # 新的录音数据长度
        N_step = int(math.floor(ny / pt_step))  # 新的录音数据包含的能量点数

        # 输出当前录音数据的能量点数和录音长度
        # print "!!!!"
        # print N_step
        # print ny
        # print "!!!!"

        # for循环，求新的2.5s录音数据的所有能量点
        # for i_step in range(1,N_step):
        for i_step in range(0, N_step):  # 循环从0开始，次数为 N_step
            abs_y1_smth = np.abs(y_next[i_step * pt_step: (i_step + 1) * pt_step])
            eng_step.append(np.sum(abs_y1_smth))  # 将新计算点加入list的尾部

        # 输出所求新的能量点
        # print "****eng_step"
        # print(eng_step)

        # 求新的平均能量点
        # avrg_eng_step.append(np.sum(eng_step[((n_avrg_eng_step-1)*5):(n_eng_step+4)])/ avrg_time)
        avrg_eng_step.append(np.sum(eng_step[n_avrg_eng_step * 5:(n_avrg_eng_step + 1) * 5]) / avrg_time)

        print "#######"
        print avrg_eng_step[-1]
        print background_eng
        print "&&&&&&&"
        # 输出新求的能量点和平均能量点
        # print "****"
        # print eng_step[(n_avrg_eng_step * 5):(n_eng_step + 4)]# 新求的能量点
        # print "****avrg_eng_step"
        # print (avrg_eng_step)# 新求的平均能量点

        # 判断新点报警状态
        # if判断，决定新的平均能量点的报警状态
        if alarm_sign == 0:  # 当前是非报警状态
            # if (avrg_eng_step[n_avrg_eng_step] > (avrg_eng_step[n_avrg_eng_step - 1] * 2)):
            # if (avrg_eng_step[n_avrg_eng_step] > (avrg_eng_step[n_avrg_eng_step-2]*5)):# 是报警点
            # if (avrg_eng_step[n_avrg_eng_step] > (background_eng*5)):# 是报警点
            if avrg_eng_step[n_avrg_eng_step] > (background_eng * 2.4):  # 是报警点
                alarm_sign = 1  # 报警

            # elif (avrg_eng_step[n_avrg_eng_step-1] > (background_eng*2)):# 是过渡点
            elif avrg_eng_step[n_avrg_eng_step] > (background_eng * 1.5):  # 是过渡点
                alarm_sign = -1  # 过渡点

            else:  # 是非报警点
                alarm_sign = 0  # 非报警

                # background_eng = avrg_eng_step[n_avrg_eng_step]
                background_eng = avrg_eng_step[n_avrg_eng_step]  # 更新背景值
                bkgrd_eng.append(background_eng)  # 增添新的背景值

        elif alarm_sign == 1:  # 当前是报警状态
            # if (avrg_eng_step[n_avrg_eng_step] > (background_eng * 2)):
            # if (avrg_eng_step[n_avrg_eng_step - 1] > (background_eng * 2)):
            # if (avrg_eng_step[n_avrg_eng_step-1] > (background_eng*5)):# 是报警点
            if avrg_eng_step[n_avrg_eng_step] > (background_eng * 2.4):  # 是报警点
                alarm_sign = 1  # 报警

            # elif (avrg_eng_step[n_avrg_eng_step-1] > (background_eng*2)):  # 是过渡点
            elif avrg_eng_step[n_avrg_eng_step] > (background_eng * 1.5):  # 是过渡点
                alarm_sign = -1  # 过渡点

            else:  # 是非报警点
                alarm_sign = 0  # 非报警

                # background_eng = avrg_eng_step[n_avrg_eng_step]
                background_eng = avrg_eng_step[n_avrg_eng_step]  # 更新背景值
                bkgrd_eng.append(background_eng)  # 增添新的背景值

        elif alarm_sign == -1:  # 当前是过渡点状态
            # if (avrg_eng_step[n_avrg_eng_step-1] > (background_eng*5)):  # 是报警点
            if avrg_eng_step[n_avrg_eng_step] > (background_eng * 2.4):  # 是报警点
                alarm_sign = 1  # 报警

            # elif (avrg_eng_step[n_avrg_eng_step-1] > (background_eng*2)):  # 是过渡点
            elif avrg_eng_step[n_avrg_eng_step] > (background_eng * 1.5):  # 是过渡点
                alarm_sign = -1  # 过渡点

            else:  # 是非报警点
                alarm_sign = 0  # 非报警

                # background_eng = avrg_eng_step[n_avrg_eng_step]
                background_eng = avrg_eng_step[n_avrg_eng_step]  # 更新背景值
                bkgrd_eng.append(background_eng)  # 增添新的背景值

                # else:
                # alarm_sign = 0
                # background_eng = avrg_eng_step[n_avrg_eng_step - 1]

        # 增添新的报警状态记录 & 报警次数累加
        alarm_all.append(alarm_sign)
        alarm_cnt += 1

        if alarm_sign == 1:
            print "AAAAAAAAAAAAA"
            robot_state.abnormal_detection = True
        else:
            robot_state.abnormal_detection = False

            # if判断，决定while循环何时终止
            # if alarm_cnt  > 20:
            # if alarm_cnt  > 10:
            # if alarm_cnt > last_time:
            #    loop_sign = 1

        robot_state.header.stamp = rospy.Time.now()
        robot_state.header.frame_id = ''
        robot_state.header.seq = seq

        pub.publish(robot_state)
        seq = seq + 1

        #print array
        rate.sleep()

if __name__ == '__main__':
    try:
        robot_state_abnormal()
    except rospy.ROSInterruptException:
        pass
