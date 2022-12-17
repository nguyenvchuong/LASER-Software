import matplotlib.pyplot as plt
import numpy as np

state = np.genfromtxt('jump_full_state.txt', delimiter=' ')

## action
#FR_x = state[:,][:,76]
#FL_x = state[:,][:,79]
RR_x = state[:,][1:800,82]
RL_x = state[:,][1:800,85]
fig1, ax1 = plt.subplots()
t = np.linspace(0,RR_x.shape[0],RR_x.shape[0])
#ax1.plot(t, FR_x, label='FR')
#ax1.plot(t, FL_x, label='FL') 
ax1.plot(t, RR_x, label='RR')  
ax1.plot(t, RL_x, label='RL') 
ax1.legend()
ax1.set_title("delta x")


## action
#FR_y = state[:,][:,77]
#FL_y = state[:,][:,80]
RR_y = state[:,][1:800,83]
RL_y = state[:,][1:800,86]
fig2, ax2 = plt.subplots()
t = np.linspace(0,RR_y.shape[0],RR_y.shape[0])
#ax2.plot(t, FR_y, label='FR')
#ax2.plot(t, FL_y, label='FL') 
ax2.plot(t, RR_y, label='RR')  
ax2.plot(t, RL_y, label='RL') 
ax2.legend()
ax2.set_title("delta y")

## action
#FR_z = state[:,][:,78]
#FL_z = state[:,][:,81]
RR_z = state[:,][1:800,84]
RL_z = state[:,][1:800,87]
fig3, ax3 = plt.subplots()

t = np.linspace(0,RR_z.shape[0],RR_z.shape[0])
#ax3.plot(t, FR_z, label='FR')
#ax3.plot(t, FL_z, label='FL') 
ax3.plot(t, RR_z, label='RR')  
ax3.plot(t, RL_z, label='RL') 
ax3.legend()
ax3.set_title("delta z")

plt.show()
