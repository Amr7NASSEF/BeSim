figure 
H=[336.88 381.34 442.25 359.25; 357.38 392	387.76	392.38;333.37	374	424.81	354.64 ;536.62	530.98	742.08	620.55 ;203.94	228.78	218.41	198.60;562.36	552	732.02	644.65;212.29	273.21	226	206.94];
bar(H)
axis([0 8 0 800 ]);
xticklabels({'Scenario 1','Scenario 2','Scenario 3','Scenario 4','Scenario 5','Scenario 6','Scenario 7','fontsize',14+2})
legend('Nominal MPC','RMPC Min-Max','RMPC LMI','GS-RMPC Min-Max')
title('Heating in Winter','fontsize',14+2);
ylabel('Heat flows [kW]','fontsize',14) 
grid on
ax = gca;
ax.FontSize = 20; 

figure
C=[55.60 0.97 15.25	31.58 ; 21.16 9.95 16.96 12.36 ;147.39	62.98	39.75	114.77;913.73	697.72	90.08	36.14;0	0.4	8.11	14.47;1020	841.55	57.22	58.64;2.74	0.68	13.01	24.28];
bar(C)
axis([0 8 0 5000 ]);
set(gca, 'YScale', 'log')
xticklabels({'Scenario 1','Scenario 2','Scenario 3','Scenario 4','Scenario 5','Scenario 6','Scenario 7','fontsize',14+2})
legend('Nominal MPC','RMPC Min-Max','RMPC LMI','GS-RMPC Min-Max')
title('Comfort Zone Violation in Winter','fontsize',14+2);
ylabel('Violation [kWh]','fontsize',14)  
grid on
ax = gca;
ax.FontSize = 20; 

figure 
HS=[49.37	55.79	74.41	59.09;70.99	75	68.28	64.11;56.65	62.52	72	65.53;54.50	59.63	78.59	74.32;40.57	37.07	67	35.93;70.60	75.63	85	92.72;44.04	42.2	70	41.09];
bar(HS)
axis([0 8 0 150 ]);
xticklabels({'Scenario 1','Scenario 2','Scenario 3','Scenario 4','Scenario 5','Scenario 6','Scenario 7','fontsize',14+2})
legend('Nominal MPC','RMPC Min-Max','RMPC LMI','GS-RMPC Min-Max')
title('Heating in Summer','fontsize',14+2);
ylabel('Heat flows [kW]','fontsize',14) 
grid on
ax = gca;
ax.FontSize = 16; 

figure
CS=[31.83	28.84	108	27.58;128.09	30	107.0	37.32;88.86	99.41	170	95.53;60.20	11.48	110	3.33;40.38	42.08	122	42.03;136.55	48.87	180	12.36;28.85	30.62	186	30.52];
bar(CS)
axis([0 8 0 200 ]);
%set(gca, 'YScale', 'log')
xticklabels({'Scenario 1','Scenario 2','Scenario 3','Scenario 4','Scenario 5','Scenario 6','Scenario 7','fontsize',14+2})
legend('Nominal MPC','RMPC Min-Max','RMPC LMI','GS-RMPC Min-Max')
title('Comfort Zone Violation in Summer','fontsize',14+2);
ylabel('Violation [kWh]','fontsize',14)  
grid on
ax = gca;
ax.FontSize = 16; 


figure
T=[90 800 36000	3000;90 800 36000	3000];
bar(T)
set(gca,'xtick',[])
axis([0.5 1.5 0 100000]);
set(gca, 'YScale', 'log')
text(0.69,70,'Nominal MPC','Color','k','FontSize',14');
text(0.86,600,'RMPC Min-Max','Color','k','FontSize',14');
text(1.055,33000,'RMPC LMI','Color','k','FontSize',14');
text(1.22,2500,'GS-RMPC Min-Max','Color','k','FontSize',14');
grid on
legend('Nominal MPC','RMPC Min-Max','RMPC LMI','GS-RMPC Min-Max')
title('Computational Time','fontsize',14+2);
ylabel('Seconds','fontsize',14)  

