# -*- coding: utf-8 -*-
from pyaudio import PyAudio, paInt16
import numpy as np
from datetime import datetime
import math
import wave


background_eng = 0


Fs = 11026    # 取样频率

pt_step=math.floor(Fs/2)

delta_time=2.5
avrg_time=10
last_time=10
alarm_cnt=0

NUM_SAMPLES = int(Fs * delta_time)    # pyAudio内部缓存的块的大小
NUM_TEST = 4 * NUM_SAMPLES
# 开启声音输入
pa = PyAudio()
stream = pa.open(format=paInt16, channels=1, rate=Fs, input=True,
                frames_per_buffer = NUM_TEST)

save_count = 0
save_buffer = [] 
eng_step = []
avrg_eng_step = []
alarm_all = []

string_audio_data = stream.read(NUM_TEST)
y1 = np.fromstring(string_audio_data, dtype=np.short)

ny = len(y1)
N_step = int(math.floor(ny/pt_step))

for i_step in range(0,N_step):
    abs_y1_smth = np.abs(y1[i_step * pt_step : (i_step + 1) * pt_step])
    eng_step.append(np.sum(abs_y1_smth))


for i_avrg in range(0,int((math.floor(N_step / 5) - 1))):
#    avrg_eng_step.append(np.sum(eng_step[(i_avrg * 5):(i_avrg + 1) * 5]) / avrg_time)
    avrg_eng_step.append(np.sum(eng_step[(i_avrg * 5):(i_avrg + 1) * 5 + 4]) / avrg_time)

if (avrg_eng_step[2]>(avrg_eng_step[0]*2)):
    alarm_sign=1
    background_eng = avrg_eng_step[0]
else:
    alarm_sign=0

alarm_all.append(alarm_sign)
alarm_cnt = alarm_cnt + 1

loop_sign=0

print "****eng_step"
print(eng_step)
print "****avrg_eng_step"
print (avrg_eng_step)

# while True:
    # 读入NUM_SAMPLES个取样
    # string_audio_data = stream.read(NUM_SAMPLES)
    # print(string_audio_data,)
    # # 将读入的数据转换为数组
    # audio_data = np.fromstring(string_audio_data, dtype=np.short)
    # # 计算大于LEVEL的取样的个数
    # print np.max(audio_data)
    # save_buffer.append(string_audio_data)
    # # 如果个数大于COUNT_NUM，则至少保存SAVE_LENGTH个块
    # print(save_buffer)
    # filename = datetime.now().strftime("%Y-%m-%d_%H_%M_%S") + ".wav"
    # save_wave_file(filename, save_buffer)
    # save_buffer = []
    # print filename, "saved"
while (loop_sign == 0):

    n_eng_step = len(eng_step)
    n_avrg_eng_step = len(avrg_eng_step)

    string_audio_data = stream.read(NUM_SAMPLES)
    y_next = np.fromstring(string_audio_data, dtype=np.short)

    ny = len(y_next)
    N_step = int(math.floor(ny / pt_step))

    print "!!!!"
    print N_step
    print ny
    print "!!!!"

#    for i_step in range(1,N_step):
    for i_step in range(0,N_step):
        abs_y1_smth = np.abs(y_next[i_step * pt_step: (i_step + 1) * pt_step])
        eng_step.append(np.sum(abs_y1_smth))

    print "****eng_step"
    print(eng_step)

#    avrg_eng_step.append(np.sum(eng_step[((n_avrg_eng_step-1)*5):(n_eng_step+4)])/ avrg_time)
    avrg_eng_step.append(np.sum(eng_step[(n_avrg_eng_step * 5):(n_eng_step + 4)])/ avrg_time)
    print "****"
    print eng_step[(n_avrg_eng_step * 5):(n_eng_step + 4)]
    print "****avrg_eng_step"
    print (avrg_eng_step)
    if alarm_sign == 0 :
#        if (avrg_eng_step[n_avrg_eng_step] > (avrg_eng_step[n_avrg_eng_step - 1] * 2)):
        if (avrg_eng_step[n_avrg_eng_step] > (avrg_eng_step[n_avrg_eng_step - 2] * 2)):
            alarm_sign = 1
#            background_eng = avrg_eng_step[n_avrg_eng_step]
            background_eng = avrg_eng_step[n_avrg_eng_step - 2]

        else:
            alarm_sign = 0

    elif alarm_sign == 1:
#        if (avrg_eng_step[n_avrg_eng_step] > (background_eng * 2)):
        if (avrg_eng_step[n_avrg_eng_step - 1] > (background_eng * 2)):
            alarm_sign = 1
        else:
            alarm_sign = 0
            background_eng = avrg_eng_step[n_avrg_eng_step - 1]

    alarm_all.append(alarm_sign)
    alarm_cnt = alarm_cnt + 1

#    if alarm_cnt  > 20:
    if alarm_cnt  > 10:
        loop_sign = 1


print(alarm_all)
