#!/usr/bin/env python



# import rospy
import serial
import sys
import string
import operator
import numpy as np
import math
import matplotlib.pyplot as plt




# from sensor_msgs.msg import NavSatFix
# #print("import Nav")
# from work1.msg import comb
# #print("import comb")



glv_Re = 6378137
glv_f = 1 / 298.25
glv_e = math.sqrt(2 * glv_f - glv_f ** 2)
glv_e2 = glv_e ** 2
glv_Rp = (1 - glv_f) * glv_Re
glv_ep = math.sqrt(glv_Re ** 2 + glv_Rp ** 2) / glv_Rp
glv_ep2 = glv_ep ** 2
glv_wie = 7.2921151467e-5
glv_g0 = 9.7803267714
glv_mg = 1.0e-3 * glv_g0
glv_ug = 1.0e-6 * glv_g0

n = 0

f1 = open('/home/exbot/imu_data.txt','a')
f2 = open('/home/exbot/123.txt','r')

def Qnorm(Q_con):

    Q_norm = np.linalg.norm(Q_con)
    return Q_con / Q_norm


def text_data():

    line = f2.readline()
    if line:
        words = string.split(line, ",")
        GyroX = string.atof(words[0])
        GyroY = string.atof(words[1])
        GyroZ = string.atof(words[2])
        AccX = string.atof(words[3])
        AccY = string.atof(words[4])
        AccZ = string.atof(words[5])
        print (GyroX, GyroY, GyroZ, AccX, AccY, AccZ)
        return (GyroX, GyroY, GyroZ, AccX, AccY, AccZ)
    else:
        return (10000,0,0,0,0,0)

def raw_data():
    global n
    while 1:
        if ser.read() != '$':
            continue
        line0 = ser.readline()
        cs = line0[-4:-2]
        cs1 = int(cs, 16)
        cs2 = checksum(line0)
        # print("cs1,cs2", cs1, cs2)
        if cs1 != cs2:
            continue
        line = line0.replace("GTIMU,", "")
        line = line.replace("\r\n", "")
        # if "\x00" in line:
        #     continue
        if not string.find('*', line):
            continue
        line = line.replace("*", ",")

        words = string.split(line, ",")

        # print(words)
        GPSWeek = string.atoi(words[0])
        GPSTime = string.atof(words[1])
        GyroX = string.atof(words[2])
        GyroY = string.atof(words[3])
        GyroZ = string.atof(words[4])
        AccX = string.atof(words[5])
        AccY = string.atof(words[6])
        AccZ = string.atof(words[7])
        Tpr = string.atof(words[8])

        s = str(GyroX) + ',' + str(GyroY) + ',' + str(GyroZ) + ',' + str(AccX) + ',' + str(AccY)+ ',' + str(AccZ) + '\n'
        f1.write(s)
        print (n)
        n = n + 1

        return (GyroX, GyroY, GyroZ, AccX, AccY, AccZ)

def coarse_align(GyroX,GyroY,GyroZ,AccX,AccY,AccZ,g,wie,L):
    T31 = AccX /g
    T32 = AccY /g
    T33 = AccZ /g
    T21 = (GyroX - T31 * math.sin(L)) / (wie * math.cos(L))
    T22 = (GyroY - T32 * math.sin(L)) / (wie * math.cos(L))
    T23 = (GyroZ - T33 * math.sin(L)) / (wie * math.cos(L))
    T11 = T22*T33 - T23*T32
    T12 = T23*T31 - T21*T33
    T13 = T21*T32 - T22*T31
    C_n_b = np.matrix([[T11, T12, T13],[T21, T22, T23],[T31, T32, T33]])
    return C_n_b

def m_cross(a,b):
    a_tmp = a.reshape(1, 3)
    b_tmp = b.reshape(1, 3)
    r = np.cross(a_tmp, b_tmp).reshape(3, 1)
    return r

def global_vip():
    global glv_Re,glv_f,glv_e,glv_e2,glv_Rp,glv_ep,glGyroXv_ep2,glv_wie,glv_g0,glv_mg,glv_ug

def earth(pos, vn):
    # global_vip()
    sl=math.sin(pos[0])
    cl=math.cos(pos[0])
    tl=sl/cl
    sl2=sl*sl
    sl4=sl2*sl2
    wien = glv_wie*np.array([[cl],[0],[-sl]])
    Rx = glv_Re*(1-2*glv_e2+3*glv_e2*sl2)+pos[2]
    Ry = glv_Re*(1+glv_e2)*sl2+pos[2]
    wenn = np.array([[0],[0],[0]])
    wenn[0]=vn[1] / (Ry + pos[2])
    wenn[1]=-vn[0] / (Rx + pos[2])
    wenn[2]=-vn[1] * tl / (Ry + pos[2])
    g = glv_g0*(1+5.27094e-3*sl2+2.32718e-5*sl4)-3.086e-6*pos[2]
    gn = np.array([[0],[0],[-g]])
    return (wien, wenn, Rx, Ry, gn)

def qmulti(p, q):
    q = np.dot(np.array([[p[0], -p[1], -p[2], -p[3]],
                   [p[1], p[0], -p[3], p[2]],
                   [p[2], p[3], p[0], -p[1]],
                   [p[3], -p[2], p[1], p[0]]]),q)
    return q


def att2C_b_n(yaw, pitch, roll):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    C_b_n = np.matrix([[cy * cr - sr * sp, -sy * cp, sr * cy + cr * sy * sp],
                       [cr * sy + sr * cy * sp, cy * cp, sr * sy - cr * cy * sp],
                       [-sr * cp, sp, cr * cp]])
    return C_b_n


def Q2C_b_n(Q):

    C_b_n = np.matrix([[Q[0] ** 2 + Q[1] ** 2 - Q[2] ** 2 - Q[3] ** 2, 2 * (Q[1] * Q[2] - Q[0] * Q[3]),
                        2 * (Q[1] * Q[3] + Q[0] * Q[2])],
                       [2 * (Q[1] * Q[2] + Q[0] * Q[3]), 1 - 2 * (Q[1] ** 2 + Q[3] ** 2),
                        2 * (Q[2] * Q[3] - Q[0] * Q[1])],
                       [2 * (Q[1] * Q[3] - Q[0] * Q[2]), 2 * (Q[2] * Q[3] + Q[0] * Q[1]),
                        Q[0] ** 2 - Q[1] ** 2 - Q[2] ** 2 + Q[3] ** 2]])
    return C_b_n


def C_b_n2Q(C_b_n):
    Q_1 = 0.5 * math.sqrt(abs(1 + C_b_n[0, 0] - C_b_n[1, 1] - C_b_n[2, 2]))
    Q_2 = 0.5 * math.sqrt(abs(1 - C_b_n[0, 0] + C_b_n[1, 1] - C_b_n[2, 2]))
    Q_3 = 0.5 * math.sqrt(abs(1 - C_b_n[0, 0] - C_b_n[1, 1] + C_b_n[2, 2]))
    Q_0 = 0.5 * math.sqrt(abs(1 + C_b_n[0, 0] + C_b_n[1, 1] + C_b_n[2, 2]))

    Q_1 = Q_1 * np.sign(C_b_n[2, 1] - C_b_n[1, 2])
    Q_2 = Q_2 * np.sign(C_b_n[0, 2] - C_b_n[2, 0])
    Q_3 = Q_3 * np.sign(C_b_n[1, 0] - C_b_n[0, 1])

    Q = np.array([Q_0, Q_1, Q_2, Q_3])
    return Q


def r2q(r):

    fi = np.linalg.norm(r)

    q = np.array(
        [math.cos(fi / 2), r[0] / fi * math.sin(fi / 2), r[1] / fi * math.sin(fi / 2), r[2] / fi * math.sin(fi / 2)])
    return q

def checksum(nmea_str0):
    nmea_str = nmea_str0[0:-5]
    return reduce(operator.xor, map(ord, nmea_str), 0)


# port = '/dev/ttyUSB0'
#
# try:
#     ser = serial.Serial(port=port, baudrate=115200, timeout=1)
# except serial.serialutil.SerialException:
#     print("IMU not found at port " + port + ". Did you specify the correct port in the launch file?")
#     # exit
#     sys.exit(0)


rpd = math.pi / 180
e = 1 / 298.3
R_e = 6378254
R_p = 6356803
w_ie = 7.2722e-005



# vec_Position_m1 = [h_X_e0 h_Y_n0 h_Z_u0]'
# H_m1 = h_H0
# L_m1 = h_L0
# J_m1 = h_J0
# vec_V_m1 = [true_v_x_rec(1) true_v_y_rec(1) true_v_z_rec(1)]'

vec_Position_m1 = math.pi/180*np.array([[36],[120],[60]])
H_m1 = 0    #High
L_m1 = 0    #Lattitude
J_m1 = 0    #lontitude
vec_V_m1 = np.array([[1],[0],[0]])

vec_rot_n = np.array([[0], [0], [0]])
# J=h_J
# h_L = 45 * rpd
# H=h_H

vec_V = vec_V_m1
# vec_Position=[h_X_e h_Y_n h_Z_u]'
vec_Position = vec_Position_m1

dRn_m = np.array([0, 0, 0])

Ts = 0.01

vec_w1 = np.array([[0], [0], [0]])
vec_f1 = np.array([[0], [0], [0]])

GyroX_Sum = 0
GyroY_Sum = 0
GyroZ_Sum = 0
AccX_Sum = 0
AccY_Sum = 0
AccZ_Sum = 0

for i in range(1, 100):
    (GyroX, GyroY, GyroZ, AccX, AccY, AccZ) = text_data()
    GyroX_Sum = GyroX_Sum + GyroX
    GyroY_Sum = GyroY_Sum + GyroX
    GyroZ_Sum = GyroZ_Sum + GyroX
    AccX_Sum = AccX_Sum + AccX
    AccY_Sum = AccY_Sum + AccY
    AccZ_Sum = AccZ_Sum + AccZ

GyroX = GyroX_Sum / 100
GyroY = GyroY_Sum / 100
GyroZ = GyroZ_Sum / 100
AccX = AccX_Sum / 100
AccY = AccY_Sum / 100
AccZ = AccZ_Sum / 100

(vec_w_n_ie, vec_w_n_en, R_m, R_n, vec_g) = earth(vec_Position, vec_V)

C_n_b = coarse_align(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, -vec_g[2, 0], glv_wie, vec_Position[0, 0])
C_b_n = np.transpose(C_n_b)
Q = C_b_n2Q(C_b_n)

#
pitch222_rec = []
yaw222_rec = []
roll222_rec = []
vx_rec = []
vy_rec = []
vz_rec = []
px_rec = []
py_rec = []
pz_rec = []

# ser.flush()

seq = 0

while 1:

    # $GTIMU,0,1148.980,1.3391,-0.6484,-0.7041,0.0040,0.0048,1.0018,48.1*5B#

    (GyroX, GyroY, GyroZ, AccX, AccY, AccZ) = text_data()
    if GyroX == 10000:
        break

    (vec_w_n_ie, vec_w_n_en, R_m, R_n, vec_g) = earth(vec_Position, vec_V)

    # R_m = R_e * (1 - e * e) / (1 - e * e * math.sin(L_m1) * math.sin(L_m1)) ^ (3 / 2)
    # R_n = R_e / (1 - e * e * math.sin(L_m1) * math.sin(L_m1)) ^ (1 / 2)
    # g = 9.7803267714 * (
    #     1 + 0.00527094 * math.sin(L_m1) * math.sin(L_m1) + 0.0000232718 * math.sin(L_m1) * math.sin(L_m1) * math.sin(
    #         L_m1) * math.sin(
    #         L_m1)) - 0.3086 * 0.00001 * H_m1
    # vec_g = np.array([[0], [0], [-g]])

    w_bx_ib = GyroX
    w_by_ib = GyroY
    w_bz_ib = GyroZ

    f_bx = AccX
    f_by = AccY
    f_bz = AccZ

    vec_w = np.array([[w_bx_ib], [w_by_ib], [w_bz_ib]])
    vec_f = np.array([[f_bx], [f_by], [f_bz]])

    # vec_w_n_ie = np.array([[0], [w_ie * math.cos(L_m1)], [w_ie * math.sin(L_m1)]])
    # vec_w_n_en = np.array([[-vec_V_m1[2] / (R_m + H_m1)], [vec_V_m1[1] / (R_n + H_m1)],
    #                        [vec_V_m1[1] * math.tan(L_m1) / (R_n + H_m1)]])

    Q_con = np.array([Q[0], -Q[1], -Q[2], -Q[3]])
    C_n_b = Q2C_b_n(Qnorm(Q_con))
    vec_w = vec_w - C_n_b * (vec_w_n_ie + vec_w_n_en)

    w_a = vec_w1
    w_b = (vec_w - vec_w1) / Ts / 2

    r = w_a * Ts + w_b * Ts * Ts + 1 / 6 * Ts * Ts * Ts * m_cross(w_a, w_b)

    fi = np.linalg.norm(r)
    if fi > 0:
        q = r2q(r)
        Q = qmulti(Q, q)
        C_b_n = Q2C_b_n(Qnorm(Q))

    pitch = math.asin(C_b_n[2, 1])
    roll = math.atan(-C_b_n[2, 0] / C_b_n[2, 2])
    yaw = math.atan(-C_b_n[0, 1] / C_b_n[1, 1])

    vec_dV_g_corm = (vec_g - m_cross((2 * vec_w_n_ie + vec_w_n_en), vec_V_m1)) * Ts

    vec_dV = (vec_f1 + vec_f) * Ts / 2
    vec_dgyro_out = (vec_w1 + vec_w) * Ts / 2
    vec_dV_rotm = 1 / 2 * m_cross(vec_dgyro_out, vec_dV)

    v_A = vec_f1
    v_B = (vec_f - vec_f1) / Ts / 2
    vec_dV_sculm = Ts * Ts * Ts / 6 * (m_cross(w_a, v_B) + m_cross(v_A, w_b))

    vec_dV_sfm = vec_dV + vec_dV_rotm + vec_dV_sculm
    vec_V = vec_V_m1 + C_b_n * vec_dV_sfm + vec_dV_g_corm

    dR_n = (vec_V_m1 + vec_V) * Ts / 2
    vec_Position = vec_Position_m1 + dR_n

    H = H_m1 + dR_n[2]
    L = L_m1 + dR_n[1] / (R_m + H)
    J = J_m1 + dR_n[0] / (R_n + H) / math.cos(L)

    vec_Position_m1 = vec_Position
    H_m1 = H
    L_m1 = L
    J_m1 = J
    vec_V_m1 = vec_V
    vec_w1 = vec_w
    vec_f1 = vec_f

    pitch222_rec.append(pitch)
    yaw222_rec.append(yaw)
    roll222_rec.append(roll)
    vx_rec.append(vec_V[0])
    vy_rec.append(vec_V[1])
    vz_rec.append(vec_V[2])
    px_rec.append(vec_Position[0,0])
    py_rec.append(vec_Position[1,0])
    pz_rec.append(vec_Position[2,0])

print ('!!!')
plt.figure(1)
plt.plot(px_rec,py_rec)
while 1:
    print '1'