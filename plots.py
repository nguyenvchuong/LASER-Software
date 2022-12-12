import matplotlib.pyplot as plt
import numpy as np

# time in ms (time interval of interest)
t = np.linspace(0,300,300) # for 1 MPC gait cycle
# t = np.linspace(0,99,99) # for 1 QP gait cycle

# foot force
FootForce = np.genfromtxt('QPsolution.txt', delimiter=' ')

f_0 = FootForce[:,][:,0]
f_1 = FootForce[:,][:,1]
# f_2 = FootForce[:,][:,2]
# f_3 = FootForce[:,][:,3]

fig, ax = plt.subplots()
ax.plot(t, f_0, label='v_Des') # leg 0
ax.plot(t, f_1, label='v_act') # leg 1
# ax.plot(t, f_2, label='Leg 2') # leg 2
# ax.plot(t, f_3, label='Leg 3') # leg 3

ax.set_xlabel('Time [ms]')
ax.set_ylabel('v')
ax.set_title("foot velocity")
ax.legend()
#plt.show()

# pDes in robot frame
pDes = np.genfromtxt('b_des_z.txt', delimiter=' ')
fig1, ax1 = plt.subplots()
# pDes command
p_0 = pDes[:,][:,0]
p_1 = pDes[:,][:,1]
# p_2 = pDes[:,][:,2]
# p_3 = pDes[:,][:,3]

# ax1.plot(t, p_0, label='Leg 0') # leg 0
# ax1.plot(t, p_1, label='Leg 1') # leg 1
# ax1.plot(t, p_2, label='Leg 2') # leg 2
# ax1.plot(t, p_3, label='Leg 3') # leg 3

ax1.plot(t, p_0, label='p_z_cmd')
ax1.plot(t, p_1, label='p_z_act')
#ax1.plot(t, p_2, label='z')

# pDes command and actual p
# pCmd = pDes[:,][:,0]
# pAct = pDes[:,][:,1]

# ax1.plot(t, pCmd, label='p_Des')
# ax1.plot(t, pAct, label='p_Actual')

ax1.set_xlabel('Time [ms]')
ax1.set_ylabel('Foot Pos')
ax1.set_title(" foot pos (foot 1, leg frame)")
ax1.legend()

# pDes in world frame
# pDes_w = np.genfromtxt('footswing_world.txt', delimiter=' ')

# p_0_w = pDes_w[:,][:,0]
# p_1_w = pDes_w[:,][:,1]
# p_2_w = pDes_w[:,][:,2]
#p_3_w = pDes_w[:,][:,3]

# fig2, ax2 = plt.subplots()
# ax2.plot(t, p_0_w, label='x') # leg 0
# ax2.plot(t, p_1_w, label='y') # leg 1
# ax2.plot(t, p_2_w, label='z') # leg 2
# ax2.plot(t, p_3_w, label='Leg 3') # leg 3

# ax2.set_xlabel('Time [ms]')
# ax2.set_ylabel('Foot Pos ')
# ax2.set_title("foot des")
# ax2.legend()

# #rpy plot
rpy = np.genfromtxt('ori.txt', delimiter=' ')
r = rpy[:,][:,0]
p = rpy[:,][:,1]
y = rpy[:,][:,2]

fig3, ax3 = plt.subplots()
ax3.plot(t, r, label='hip') # leg 0
ax3.plot(t, p, label='thigh') # leg 1
ax3.plot(t, y, label='calf') # leg 2

ax3.set_xlabel('Time [ms]')
ax3.set_ylabel('pos')
ax3.set_title("Actual Joint torque (leg 1)")
ax3.legend()

taucmd = np.genfromtxt('ori.txt', delimiter=' ')
h = taucmd[:,][:,0]
th = taucmd[:,][:,1]
ca = taucmd[:,][:,2]

fig4, ax4 = plt.subplots()
ax4.plot(t, h, label='hip') # leg 0
ax4.plot(t, th, label='thigh') # leg 1
ax4.plot(t, ca, label='calf') # leg 2

ax4.set_xlabel('Time [ms]')
ax4.set_ylabel('pos')
ax4.set_title("Joint torque cmd (leg 1)")
ax4.legend()

plt.show()