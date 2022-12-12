import scipy.io
import numpy as np

data = scipy.io.loadmat("jumping_up_platform_A1Robot_1ms_h50_d60.mat")
data = {k:v for k, v in data.items() if k[0] != '_'}

parameter = data.keys()

for i in parameter:
	np.savetxt(("data_{}.csv".format(i)), data.get(i), delimiter=",")
