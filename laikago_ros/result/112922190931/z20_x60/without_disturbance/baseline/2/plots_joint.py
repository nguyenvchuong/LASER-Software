import matplotlib.pyplot as plt
import numpy as np

state = np.genfromtxt('jump_full_state.txt', delimiter=' ')

## leg 3

fig1, ax1 = plt.subplots()
h = state[:,][1:1200,12]
th = state[:,][1:1200,13]
ca = state[:,][1:1200,14]

t = np.linspace(0,h.shape[0],h.shape[0])
ax1.plot(t, h*180/3.14, label='hip')  
ax1.plot(t, th*180/3.14, label='thigh') 
ax1.plot(t, ca*180/3.14, label='calf') 
ax1.legend()
ax1.set_title("joint position RR")


h_fr = state[:,][1:2000,92]
t_fr = state[:,][1:2000,93]
c_fr = state[:,][1:2000,94]

fig2, ax2 = plt.subplots()
# torque send to motor
t = np.linspace(0,h_fr.shape[0],h_fr.shape[0])

ax2.plot(t, h_fr, label='hip') # 
ax2.plot(t, t_fr, label='thigh') # 
ax2.plot(t, c_fr, label='calf') # 
ax2.legend()
ax2.set_title("Torque for front right leg")



plt.show()
