import matplotlib.pyplot as plt
import numpy as np

state = np.genfromtxt('jump_full_state.txt', delimiter=' ')

# note the order of foot position and foot velocity

## leg 3
h = state[:,][1:1200,12]
th = state[:,][1:1200,13]
ca = state[:,][1:1200,14]
fig1, ax1 = plt.subplots()
t = np.linspace(0,h.shape[0],h.shape[0])
ax1.plot(t, h*180/3.14, label='hip')  
ax1.plot(t, th*180/3.14, label='thigh') 
ax1.plot(t, ca*180/3.14, label='calf') 
ax1.legend()
ax1.set_title("joint position RR")

plt.show()
