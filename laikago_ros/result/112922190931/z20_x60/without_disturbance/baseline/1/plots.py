import matplotlib.pyplot as plt
import numpy as np

state = np.genfromtxt('jump_full_state.txt', delimiter=' ')
fig1, ax1 = plt.subplots()
# actual CoM position 
p_x = state[:,][1:2000,0]
p_y = state[:,][1:2000,1]
p_z = state[:,][1:2000,2]

t = np.linspace(0,p_x.shape[0],p_x.shape[0])

ax1.plot(t, p_x*1.1, label='x') # p_x
ax1.plot(t, p_y, label='y') # p_y
ax1.plot(t, p_z, label='z') # p_z
ax1.legend()
ax1.set_title("CoM position")

##rpy plot

r = state[:,][1:2000,3]
p = state[:,][1:2000,4]
y = state[:,][1:2000,5]

fig3, ax3 = plt.subplots()
ax3.plot(t, r*180/3.14, label='roll')
ax3.plot(t, p*180/3.14, label='pitch') 
ax3.plot(t, y*180/3.14, label='yaw')  
ax3.legend()
ax3.set_title("rpy")

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

## footforce
FR = state[:,][1:2000,72]
FL = state[:,][1:2000,73]
RR = state[:,][1:2000,74]
RL = state[:,][1:2000,75]
fig4, ax4 = plt.subplots()
ax4.plot(t, FR, label='FR')
ax4.plot(t, FL, label='FL') 
ax4.plot(t, RR, label='RR')  
ax4.plot(t, RL, label='RL') 
ax4.legend()
ax4.set_title("foot force")


plt.show()
