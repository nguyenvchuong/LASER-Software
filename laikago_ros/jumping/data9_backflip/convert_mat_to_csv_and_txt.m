clear
load('backflipFull_A1_1ms_h0_d-50_full_state.mat')
csvwrite('backflipFull_A1_1ms_h0_d-50_full_state.csv',data)
dlmwrite('backflipFull_A1_1ms_h0_d-50_full_state.txt',data)

% save('jumpingFull_A1_1ms_h30_d70_full_state.txt','data','-ascii')