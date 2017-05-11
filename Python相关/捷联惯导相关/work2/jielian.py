# -*- coding: utf-8 -*-
"""
Created on Fri May 27 14:27:45 2016

@author: exbot
"""

import numpy as np
import math


def qmulti(p, q):
    q = np.matrix([[p[0], -p[1], -p[2], -p[3]],
                   [p[1], p[0], -p[3], p[2]],
                   [p[2], p[3], p[0], -p[1]],
                   [p[3], -p[2], p[1], p[0]]]) * q
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
    # 由四元数生成矩阵C_b_n
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

    Q = np.array([Q_0, Q_1, Q_2, Q_3])  # 四元数的值 304页
    return Q


def r2q(r):
    # 求等效旋转矢量的模，即围绕定轴转过的角度。
    fi = np.linalg.norm(r)
    # 分别计算四元数的四个元素：
    q = np.array(
        [math.cos(fi / 2), r[0] / fi * math.sin(fi / 2), r[1] / fi * math.sin(fi / 2), r[2] / fi * math.sin(fi / 2)])
    return q


# 初始化
rpd = math.pi / 180
e = 1 / 298.3  # 地球扁率
R_e = 6378254  # 半长轴
R_p = 6356803  # 半短轴
w_ie = 7.2722e-005  # 地球自转角速度

vec_w1 = np.array([[0], [0], [0]])
vec_f1 = np.array([[0], [0], [0]])
# 初始姿态角
yaw = 0
pitch = 0
roll = 0
C_b_n = att2C_b_n(yaw, pitch, roll)  # 由初始姿态角构造初始姿态矩阵
Q = C_b_n2Q(C_b_n)  # 由初始姿态矩阵构造四元数的初始值

# vec_Position_m1 = [h_X_e0 h_Y_n0 h_Z_u0]'
# H_m1 = h_H0
# L_m1 = h_L0
# J_m1 = h_J0
# vec_V_m1 = [true_v_x_rec(1) true_v_y_rec(1) true_v_z_rec(1)]'

vec_Position_m1 = np.array([[0],[0],[0]])
H_m1 = 0
L_m1 = 0
J_m1 = 0
vec_V_m1 = np.array([[0],[0],[0]])

vec_rot_n = np.array([[0], [0], [0]])  # 导航坐标系旋转矢量初始值
# J=h_J
h_L = 45 * rpd
# H=h_H                #位置初始值

vec_V = np.array([[0], [0], [0]])  # 速度向量初始值
# vec_Position=[h_X_e h_Y_n h_Z_u]'
vec_Position = np.array([[0], [0], [0]])  # 位置初始值

dRn_m = np.array([0, 0, 0])

Ts = 0.02
#
pitch222_rec = []
yaw222_rec   = []
roll222_rec  = []

while 1:
    # 地球相关参数计算
    R_m = R_e * (1 - e * e) / (1 - e * e * math.sin(L_m1) * math.sin(L_m1)) ^ (3 / 2) # 子午圈曲率半径
    R_n = R_e / (1 - e * e * math.sin(L_m1) * math.sin(L_m1)) ^ (1 / 2) # 卯酉圈曲率半径
    g = 9.7803267714 * (1 + 0.00527094 * math.sin(L_m1) * math.sin(L_m1) + 0.0000232718 * math.sin(L_m1) * math.sin(L_m1) * math.sin(L_m1) * math.sin(
        L_m1)) - 0.3086 * 0.00001 * H_m1 # 来自文献000重力加速度
    vec_g = np.array([[0],[0],[-g]])   #构造地球重力矢量

    w_bx_ib = 1
    w_by_ib = 1
    w_bz_ib = 1

    f_bx = 1
    f_by = 1
    f_bz = 1

    vec_w = np.array([[w_bx_ib],[w_by_ib],[w_bz_ib]])   #对陀螺输出采样
    vec_f = np.array([[f_bx],[f_by],[f_bz]])            #对加速度输出采样
    # 对陀螺数据进行导航坐标系的旋转补偿
    vec_w_n_ie = np.array([[0],[w_ie * math.cos(L_m1)],[w_ie * math.sin(L_m1)]])    #地球相对于惯性坐标系的自转角速度在导航坐标系中的投影。书231页
    vec_w_n_en = np.array([[-vec_V_m1[2] / (R_m + H_m1)],[vec_V_m1[1] / (R_n + H_m1)],[vec_V_m1[1] * math.tan(L_m1) / (R_n + H_m1)]])   #导航坐标系沿地球表面移动导致的旋转角速度
    # 利用四元数Q的共轭，计算由导航坐标系到机体坐标系的旋转矩阵
    Q_con = np.array(Q[1],-Q[2],-Q[3],-Q[4])
    C_n_b = Q2C_b_n(Q_con)
    vec_w = vec_w - C_n_b * (vec_w_n_ie + vec_w_n_en) # 对陀螺输出数据进行补偿
    
    # 直线拟合更新周期内的角速度
    w_a = vec_w1
    w_b = (vec_w - vec_w1) / Ts / 2
    r = w_a * Ts + w_b * Ts * Ts + 1 / 6 * Ts * Ts * Ts * np.cross(w_a, w_b) # 计算机体旋转的等效旋转矢量：P315，（9.3.37）

    fi = np.linalg.norm(r)
    if fi > 0:                 #防止0作为除数,检查旋转矢量的模是否大于0
        q=r2q(r) # 由等效旋转矢量构造姿态变化四元数：
        Q=qmulti(Q, q) # 姿态四元数更新                            Q=qmulti(Q_m1, q)
        C_b_n=Q2C_b_n(Q) # 由姿态四元数构造姿态矩阵      out

    # 给出姿态的更新数据（控制天线姿态稳定的必要参数）
    pitch=math.asin( C_b_n(3, 2) ) # 俯仰               out
    roll=math.atan( -C_b_n(3, 1) / C_b_n(3, 3) ) # 横滚               out
    yaw=math.atan( -C_b_n(1, 2) / C_b_n(2, 2) ) # 偏航，航向角       out
    # ------------------------速度解算：------------------------                       坐标转换应尝试直接使用四元数计算，或许能提高精度
    # # 计算有害加速度引起的速度补偿量：vec_dV_g_cormvec_V为最近的一次速度更新值，vec_w_en也利用最近的速度值计算
    # vec_rot_n=vec_rot_n_m1+(vec_w_n_en+vec_w_n_ie) * T # 记录导航坐标系在修正周期内的旋转矢量（从l-1时刻到m时刻）
    
    vec_dV_g_corm = ( vec_g-np.cross( (2 * vec_w_n_ie+vec_w_n_en), vec_V_m1 ) ) * Ts # 暂时对vec_w_en进行常值积分，T为解算更新周期
    # 计算旋转效应补偿项
    vec_dV=(vec_f1+vec_f) * Ts / 2 # 本体系下本周期速度增量
    vec_dgyro_out=(vec_w1+vec_w) * Ts / 2 # 本体系下本周期角度增量
    vec_dV_rotm = 1 / 2 * np.cross(vec_dgyro_out, vec_dV) # 计算旋转效应补偿量
    # 计算划桨效应补偿项
    v_A=vec_f1
    v_B=(vec_f-vec_f1) / Ts / 2
    vec_dV_sculm=Ts * Ts * Ts / 6 * ( np.cross(w_a, v_B) + np.cross(v_A, w_b) )
    # 速度解算
    vec_dV_sfm=vec_dV+vec_dV_rotm+vec_dV_sculm
    vec_V=vec_V_m1+C_b_n * vec_dV_sfm+vec_dV_g_corm
    # ------------------------位置解算：------------------------
    dR_n=(vec_V_m1+vec_V) * Ts / 2 # 计算本解算周期内的位置变化量
    vec_Position=vec_Position_m1+dR_n # 位置更新
    
    H=H_m1+dR_n[3] # 计算当前高度
    L=L_m1+dR_n[2] / (R_m+H) # 计算当前纬度值
    J=J_m1+dR_n[1] / (R_n+H) / math.cos(L) # 计算当前经度值
    
    # 保存当前数据为上一次数据
    vec_Position_m1 = vec_Position
    H_m1 = H
    L_m1 = L
    J_m1 = J
    vec_V_m1 = vec_V
    vec_w1=vec_w
    vec_f1=vec_f
    
    pitch222_rec(m)=pitch  yaw222_rec(m)=yaw  roll222_rec(m)=roll
    vx_rec(m) = vec_V(1)   vy_rec(m) = vec_V(2)   vz_rec(m) = vec_V(3)
    px_rec(m) = vec_Position(1)   py_rec(m) = vec_Position(2)  pz_rec(m) = vec_Position(3)
