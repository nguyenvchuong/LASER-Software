import matplotlib.pyplot as plt
import numpy as np

state = np.genfromtxt('jump_full_state.txt', delimiter=' ')
ref = np.genfromtxt('jumpingFull_A1_1ms_h00_d60_full_state.txt', delimiter=',')

fig1, ax1 = plt.subplots()
# actual CoM position 
px_act = state[:,][1:1200,0]
py_act = state[:,][1:1200,1]
pz_act = state[:,][1:1200,2]

px_ref = ref[:,][1:1200,0]
py_ref = ref[:,][1:1200,1]
pz_ref = ref[:,][1:1200,2]

t = np.linspace(0,px_ref.shape[0],px_ref.shape[0])

ax1.plot(t, px_act, label='x_act', color="b") # p_x
ax1.plot(t, py_act, label='y_act', color="g") # p_y
ax1.plot(t, pz_act, label='z_act', color="r") # p_z
ax1.plot(t, px_ref, label='x_ref', color='b',linestyle='dashed') # p_x
ax1.plot(t, py_ref, label='y_ref', color='g',linestyle='dashed') # p_y
ax1.plot(t, pz_ref, label='z_ref', color='r',linestyle='dashed') # p_z
ax1.legend()
ax1.set_title("CoM position comparison")

##rpy plot

r = state[:,][1:1200,3]
p = state[:,][1:1200,4]
y = state[:,][1:1200,5]

r_ref = ref[:,][1:1200,3]
p_ref = ref[:,][1:1200,4]
y_ref = ref[:,][1:1200,5]

fig3, ax3 = plt.subplots()
ax3.plot(t, r*180/3.14, label='r_act', color="b")
ax3.plot(t, p*180/3.14, label='p_act', color="g") 
ax3.plot(t, y*180/3.14, label='y_act', color="r")  
ax3.plot(t, r_ref*180/3.14, label='r_ref', color='b',linestyle='dashed')
ax3.plot(t, p_ref*180/3.14, label='p_ref', color='g',linestyle='dashed') 
ax3.plot(t, y_ref*180/3.14, label='y_ref', color='r',linestyle='dashed') 
ax3.legend()
ax3.set_title("rpy comparison")

# ax3.set_xlabel('Time [ms]')
# # ax3.set_ylabel('pos')
# ax3.set_title("rpy")
# ax3.legend()

# taucmd = np.genfromtxt('ori.txt', delimiter=' ')
# h = taucmd[:,][:,0]
# th = taucmd[:,][:,1]
# ca = taucmd[:,][:,2]

# fig4, ax4 = plt.subplots()
# ax4.plot(t, h, label='hip') # leg 0
# ax4.plot(t, th, label='thigh') # leg 1
# ax4.plot(t, ca, label='calf') # leg 2

# ax4.set_xlabel('Time [ms]')
# ax4.set_ylabel('pos')
# ax4.set_title("Joint torque cmd (leg 1)")
# ax4.legend()

plt.show()
