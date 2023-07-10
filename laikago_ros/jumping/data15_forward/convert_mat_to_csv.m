% use both voltage and power constraints (relaxed)
clear
load('jumpingFull_A1_1ms_h20_d60_full_state.mat')
csvwrite('jumpingFull_A1_1ms_h20_d60_full_state.csv',data)
dlmwrite('jumpingFull_A1_1ms_h20_d60_full_state.txt',data)