
outdata = BeSim_2W(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);

SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =1; % 1 simulate a breakdown on all heaters on 5th for 6 hours 

outdata = BeSim_2W(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);

SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 

outdata = BeSim_2W(model_Old,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);

outdata = BeSim_2W(model_Light,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);


SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
fprintf('*** MPC Scenario 4S')
outdata = BeSim_2(model_Old,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);
set(gcf, 'name', 'MPC Scenario 4S')


SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
fprintf('*** MPC Scenario 5W')
outdata = BeSim_2(model_Light,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)
set(gcf, 'name', 'MPC Scenario 5W')


SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
fprintf('*** MPC Scenario 5S')
outdata = BeSim_2(model_Light,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)
set(gcf, 'name', 'MPC Scenario 5S')




SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
fprintf('*** MPC Scenario 6W')
outdata = BeSim_2W(model_Old,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);
set(gcf, 'name', 'MPC Scenario 6W')


SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
fprintf('*** MPC Scenario 6S')
outdata = BeSim_2W(model_Old,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);
set(gcf, 'name', 'MPC Scenario 6S')


SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
fprintf('*** MPC Scenario 7W')
outdata = BeSim_2W(model_Light,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)
set(gcf, 'name', 'MPC Scenario 7W')

SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
fprintf('*** MPC Scenario 7S')
outdata = BeSim_2W(model_Light,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)
set(gcf, 'name', 'MPC Scenario 7S')





CtrlParam.MPC.use = 0;%L2020 March 3 orignal = 1
CtrlParam.RMPC.use=1;
CtrlParam.ARMPC.use=0;
CtrlParam.RMPCLMI.use=0;

if (CtrlParam.RMPCLMI.use||CtrlParam.ARMPC.use)
    ctrl = BeCtrl_1(model_Old,model_Reno,model_Light,CtrlParam); % first model should be the plant model for LMI 
 else
    ctrl = BeCtrl(model, CtrlParam);       % construct a controller object
end

SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** RMPC Scenario 1S')
    outdata = BeSim_2(model_Reno,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);
set(gcf, 'name', 'RMPC Scenario 1S')




SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
SimParam.allDist =0;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** RMPC Scenario 2W')
    outdata = BeSim_2(model_Reno,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)
set(gcf, 'name', 'RMPC Scenario 2W')


SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
SimParam.allDist =0;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** RMPC Scenario 2S')
    outdata = BeSim_2(model_Reno,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)
set(gcf, 'name', 'RMPC Scenario 2S')


SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
fprintf('*** RMPC Scenario 5S')
outdata = BeSim_2(model_Light,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)
set(gcf, 'name', 'RMPC Scenario 5S')


SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
fprintf('*** RMPC Scenario 7S')
outdata = BeSim_2W(model_Light,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)
set(gcf, 'name', 'RMPC Scenario 7S')



SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
fprintf('*** RMPC Scenario 4W')
outdata = BeSim_2(model_Old,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);
set(gcf, 'name', 'RMPC Scenario 4W')

SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
fprintf('*** RMPC Scenario 4S')
outdata = BeSim_2(model_Old,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);
set(gcf, 'name', 'RMPC Scenario 4S')


SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
fprintf('*** RMPC Scenario 5W')
outdata = BeSim_2(model_Light,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)
set(gcf, 'name', 'RMPC Scenario 5W')


SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
fprintf('*** RMPC Scenario 5S')
outdata = BeSim_2(model_Light,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)
set(gcf, 'name', 'RMPC Scenario 5S')



SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
fprintf('*** RMPC Scenario 6W')
outdata = BeSim_2W(model_Old,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);
set(gcf, 'name', 'RMPC Scenario 6W')


SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
fprintf('*** RMPC Scenario 6S')
outdata = BeSim_2W(model_Old,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);
set(gcf, 'name', 'RMPC Scenario 6S')


SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
fprintf('*** RMPC Scenario 7W')
outdata = BeSim_2W(model_Light,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)
set(gcf, 'name', 'RMPC Scenario 7W')


SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
fprintf('*** RMPC Scenario 7S')
outdata = BeSim_2W(model_Light,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)
set(gcf, 'name', 'RMPC Scenario 7S')









CtrlParam.MPC.use = 0;%L2020 March 3 orignal = 1
CtrlParam.RMPC.use=0;
CtrlParam.ARMPC.use=1;%L2020 MUP USE 



if (CtrlParam.RMPCLMI.use||CtrlParam.ARMPC.use)
     ctrl = BeCtrl_1(model_Old,model_Reno,model_Light,CtrlParam); % first model should be the plant model for LMI 
 else
    ctrl = BeCtrl(model, CtrlParam);       % construct a controller object
end


SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** ARMPC Scenario 1W')
outdata = BeSim_2W(model_Reno,model_Reno, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);
set(gcf, 'name', 'ARMPC Scenario 1W')


SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** ARMPC Scenario 1S')
    outdata = BeSim_3(model_plant,model_Old,model_Reno,model_Light, estim, ctrl, dist, refs, SimParam);%check the shcedule gain 40 
BePlot(outdata,PlotParam);
set(gcf, 'name', 'ARMPC Scenario 1S')




SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
SimParam.allDist =0;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** ARMPC Scenario 2W')
    outdata = BeSim_3(model_Reno,model_Old,model_Reno,model_Light, estim, ctrl, dist, refs, SimParam);%check the shcedule gain 40 
BePlot(outdata,PlotParam)
set(gcf, 'name', 'ARMPC Scenario 2W')


SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
SimParam.allDist =0;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** ARMPC Scenario 2S')
    outdata = BeSim_3(model_Reno,model_Old,model_Reno,model_Light, estim, ctrl, dist, refs, SimParam);%check the shcedule gain 40 
BePlot(outdata,PlotParam)
set(gcf, 'name', 'ARMPC Scenario 2S')




SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =1; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** ARMPC Scenario 3W')
    outdata = BeSim_3(model_plant,model_Old,model_Reno,model_Light, estim, ctrl, dist, refs, SimParam);%check the shcedule gain 40 
BePlot(outdata,PlotParam)
set(gcf, 'name', 'ARMPC Scenario 3W')


SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =1; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** ARMPC Scenario 3S')
    outdata = BeSim_3(model_plant,model_Old,model_Reno,model_Light, estim, ctrl, dist, refs, SimParam);%check the shcedule gain 40 
BePlot(outdata,PlotParam)
set(gcf, 'name', 'ARMPC Scenario 3S')





SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 

CtrlParam.MPC.use = 0;%L2020 March 3 orignal = 1
CtrlParam.RMPC.use=0;
CtrlParam.ARMPC.use=1;%L2020 MUP USE 
CtrlParam.RMPCLMI.use=0;


if (CtrlParam.RMPCLMI.use||CtrlParam.ARMPC.use)
     ctrl = BeCtrl_1(model_Old,model_Reno,model_Light,CtrlParam); % first model should be the plant model for LMI 
 else
    ctrl = BeCtrl(model, CtrlParam);       % construct a controller object
end





fprintf('*** ARMPC Scenario 4W')
    outdata = BeSim_3WO(model_Old,model_Old,model_Reno,model_Light, estim, ctrl, dist, refs, SimParam);%check the shcedule gain 40 
BePlot(outdata,PlotParam);
set(gcf, 'name', 'ARMPC Scenario 4W')


SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
fprintf('*** ARMPC Scenario 4S')
    outdata = BeSim_3WO(model_Old,model_Old,model_Reno,model_Light, estim, ctrl, dist, refs, SimParam);%check the shcedule gain 40 
BePlot(outdata,PlotParam);
set(gcf, 'name', 'ARMPC Scenario 4S')


SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
fprintf('*** ARMPC Scenario 5W')
    outdata = BeSim_3WO(model_Light,model_Old,model_Reno,model_Light, estim, ctrl, dist, refs, SimParam);%check the shcedule gain 40 
BePlot(outdata,PlotParam)
set(gcf, 'name', 'ARMPC Scenario 5W')


SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
fprintf('*** ARMPC Scenario 5S')
    outdata = BeSim_3WO(model_Light,model_Old,model_Reno,model_Light, estim, ctrl, dist, refs, SimParam);%check the shcedule gain 40 
BePlot(outdata,PlotParam)
set(gcf, 'name', 'ARMPC Scenario 5S')




SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
fprintf('*** ARMPC Scenario 6W')
    outdata = BeSim_3(model_Old,model_Old,model_Reno,model_Light, estim, ctrl, dist, refs, SimParam);%check the shcedule gain 40 
BePlot(outdata,PlotParam);
set(gcf, 'name', 'ARMPC Scenario 6W')


SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
fprintf('*** ARMPC Scenario 6S')
    outdata = BeSim_3(model_Old,model_Old,model_Reno,model_Light, estim, ctrl, dist, refs, SimParam);%check the shcedule gain 40 
BePlot(outdata,PlotParam);
set(gcf, 'name', 'ARMPC Scenario 6S')


SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
fprintf('*** ARMPC Scenario 7W')
    outdata = BeSim_3(model_Light,model_Old,model_Reno,model_Light, estim, ctrl, dist, refs, SimParam);%check the shcedule gain 40 
BePlot(outdata,PlotParam)
set(gcf, 'name', 'ARMPC Scenario 7W')


SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
fprintf('*** ARMPC Scenario 7S')
    outdata = BeSim_3(model_Light,model_Old,model_Reno,model_Light, estim, ctrl, dist, refs, SimParam);%check the shcedule gain 40 
BePlot(outdata,PlotParam)
set(gcf, 'name', 'ARMPC Scenario 7S')


