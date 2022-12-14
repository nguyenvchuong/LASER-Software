import matplotlib.pyplot as plt
import numpy as np

# motor params
_gear_ratio=8.5
Kt=4/34
_R_motor = 0.346

#alpha_motor=(940*2*3.14/60-1700*2*3.14/60)/(4-0.2)=20.93

state = np.genfromtxt('jump_full_state.txt', delimiter=' ')
fig1, ax1 = plt.subplots()
# actual joint velocity 
dq_hip_front = state[:,][1:2000,24]
dq_thigh_front = state[:,][1:2000,25]
dq_calf_front = state[:,][1:2000,26]
dq_hip_rear = state[:,][1:2000,30]
dq_thigh_rear = state[:,][1:2000,31]
dq_calf_rear = state[:,][1:2000,32]

t = np.linspace(0,dq_hip_front.shape[0],dq_hip_front.shape[0])

ax1.plot(t, 2*(dq_hip_front+dq_thigh_front+dq_calf_front+dq_hip_rear+dq_thigh_rear+dq_calf_rear), label='total current') 

ax1.legend()
ax1.set_title("Total actual current")




fig2, ax2 = plt.subplots()
V_hip_front = state[:,][1:1100,36]*0.346 + state[:,][1:1100,24]
V_thigh_front = state[:,][1:1100,37]*0.346 + state[:,][1:1100,25]
V_calf_front = state[:,][1:1100,38]*0.346 + state[:,][1:1100,26]
V_hip_rear = state[:,][1:1100,42]*0.346 + state[:,][1:1100,30]
V_thigh_rear = state[:,][1:1100,43]*0.346 + state[:,][1:1100,31]
V_calf_rear = state[:,][1:1100,44]*0.346 + state[:,][1:1100,32]


t = np.linspace(0,V_hip_front.shape[0],V_hip_front.shape[0])

ax2.plot(t, V_thigh_front, label='V_thigh_front') # p_x
ax2.plot(t, V_calf_front, label='V_calf_front') # p_y
ax2.plot(t, V_thigh_rear, label='V_thigh_rear') # p_z
ax2.plot(t, V_calf_rear, label='V_calf_rear') # p_z

ax2.legend()
ax2.set_title("Estimated Voltage")

#voltage[i * 3 + j] = _controlData->_legController->commands[i].tau(j) * _R_motor / (Kt * _gear_ratio) + _controlData->_legController->data[i].qd[j] * _gear_ratio * Kt;
# in our case : _gear_ratio*Kt = 1

fig3, ax3 = plt.subplots()
V_hip_front = state[:,][1:1200,92]*0.346 + state[:,][1:1200,24]
V_thigh_front = state[:,][1:1200,93]*0.346 + state[:,][1:1200,25]
V_calf_front = state[:,][1:1200,94]*0.346 + state[:,][1:1200,26]
V_hip_rear = state[:,][1:1200,98]*0.346 + state[:,][1:1200,30]
V_thigh_rear = state[:,][1:1200,99]*0.346 + state[:,][1:1200,31]
V_calf_rear = state[:,][1:1200,100]*0.346 + state[:,][1:1200,32]


t = np.linspace(0,V_hip_front.shape[0],V_hip_front.shape[0])

ax3.plot(t, V_thigh_front, label='V_thigh_front') # p_x
ax3.plot(t, V_calf_front, label='V_calf_front') # p_y
ax3.plot(t, V_thigh_rear, label='V_thigh_rear') # p_z
ax3.plot(t, V_calf_rear, label='V_calf_rear') # p_z

ax3.legend()
ax3.set_title("Command Voltage")

plt.show()
