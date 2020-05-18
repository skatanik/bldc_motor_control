import math
import numpy as np
import matplotlib.pyplot as plt

timeline = range(360)
Beta_ang = np.zeros(360)
t_a = np.zeros(360)
t_b = np.zeros(360)
t_zero = np.zeros(360)
t1 = np.zeros(360)
t2 = np.zeros(360)
t3 = np.zeros(360)
t4 = np.zeros(360)
ch1 = np.zeros(360)
ch2 = np.zeros(360)
ch3 = np.zeros(360)
fp_t_a = np.zeros(360)
fp_t_b = np.zeros(360)
fp_t_zero = np.zeros(360)
Uv_ampl = 0.5
eq_first_part = 2/math.sqrt(3) * 4250 * Uv_ampl

fp_Vamp = 0xDDB2
fp_tConst = 0x1279a
fp_firstPart = (fp_Vamp*fp_tConst) >> 16


for i in range(360):
    sector_number = math.floor(i / 60.0) + 1
    Beta_ang[i] = i - 60.0 * (sector_number - 1)
    t_a[i] = eq_first_part * math.sin((60.0 - Beta_ang[i]) * 0.0174532925199)
    t_b[i] = eq_first_part * math.sin((Beta_ang[i]) * 0.0174532925199)
    t_zero[i] = 4250 - t_a[i] - t_b[i]

    sinB = math.floor(math.sin((Beta_ang[i]) * 0.0174532925199) * 0xffff)
    sin60mB = math.floor(math.sin((60.0 - Beta_ang[i]) * 0.0174532925199) * 0xffff)
    fp_t_a[i] = 4250*(sin60mB * fp_firstPart) >> 32
    fp_t_b[i] = 4250*(sinB * fp_firstPart) >> 32
    fp_t_zero[i] = (4250 - fp_t_a[i] - fp_t_b[i])

    if (t_zero[i] < 0):
        t_zero[i] = 0

    t1[i] = math.floor(t_a[i] + t_b[i] + t_zero[i] / 2.0)
    t2[i] = math.floor(t_b[i] + t_zero[i] / 2.0)
    t3[i] = math.floor(t_a[i] + t_zero[i] / 2.0)
    t4[i] = math.floor(t_zero[i] / 2.0)

    if(sector_number == 1):
        ch1[i] = t1[i]
        ch2[i] = t2[i]
        ch3[i] = t4[i]

    if (sector_number == 2):
        ch1[i] = t3[i]
        ch2[i] = t1[i]
        ch3[i] = t4[i]

    if (sector_number == 3):
        ch1[i] = t4[i]
        ch2[i] = t1[i]
        ch3[i] = t2[i]

    if(sector_number == 4):
        ch1[i] = t4[i]
        ch2[i] = t3[i]
        ch3[i] = t1[i]

    if(sector_number == 5):
        ch1[i] = t2[i]
        ch2[i] = t4[i]
        ch3[i] = t1[i]

    if(sector_number == 6):
        ch1[i] = t1[i]
        ch2[i] = t4[i]
        ch3[i] = t3[i]



plt.plot(timeline, ch1, ch2)
plt.plot(timeline, ch3)
# plt.plot(timeline, t_a, t_b)
# plt.plot(timeline, t_zero)
# plt.plot(timeline, fp_t_a, fp_t_b)
# plt.plot(timeline, fp_t_zero)
plt.show()