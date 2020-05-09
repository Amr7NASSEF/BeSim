CtrlParam.MPC.use = 1;%L2020 March 3 orignal = 1
if (CtrlParam.RMPCLMI.use||CtrlParam.ARMPC.use)
     ctrl = BeCtrl_1(model_Old,model_Reno,model_Light,CtrlParam); % first model should be the plant model for LMI 
 else
    ctrl = BeCtrl(model, CtrlParam);       % construct a controller object
end


SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** MPC Scenario 1W')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);


SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** MPC Scenario S1')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);

SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
SimParam.allDist =0;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** MPC Scenario 2W')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)

SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
SimParam.allDist =0;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** MPC Scenario 2S')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)

SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =1; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** MPC Scenario 3W')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)

SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =1; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** MPC Scenario 3S')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)


CtrlParam.MPC.use = 0;%L2020 March 3 orignal = 1
CtrlParam.RMPC.use=1;

if (CtrlParam.RMPCLMI.use||CtrlParam.ARMPC.use)
     ctrl = BeCtrl_1(model_Old,model_Reno,model_Light,CtrlParam); % first model should be the plant model for LMI 
 else
    ctrl = BeCtrl(model, CtrlParam);       % construct a controller object
end


SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** RMPC Scenario 1W')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);

SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** RMPC Scenario S1')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);

SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
SimParam.allDist =0;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** RMPC Scenario 2W')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)

SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
SimParam.allDist =0;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** RMPC Scenario 2S')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)

SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =1; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** RMPC Scenario 3W')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)

SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =1; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** RMPC Scenario 3S')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)


CtrlParam.MPC.use = 0;%L2020 March 3 orignal = 1
CtrlParam.RMPC.use=0;
CtrlParam.RMPCLMI.use=1; %L2020 MUP USE 


if (CtrlParam.RMPCLMI.use||CtrlParam.ARMPC.use)
     ctrl = BeCtrl_1(model_Old,model_Reno,model_Light,CtrlParam); % first model should be the plant model for LMI 
 else
    ctrl = BeCtrl(model, CtrlParam);       % construct a controller object
end


SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** RMPCLMI Scenario 1W')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);

SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** RMPCLMI Scenario S1')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam);

SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
SimParam.allDist =0;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** RMPCLMI Scenario 2W')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)

SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
SimParam.allDist =0;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** RMPCLMI Scenario 2S')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)

SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =1; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** RMPCLMI Scenario 3W')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)

SimParam.run.start =200;%S=200, W=10
SimParam.run.end = 206;%S=206, W=16 
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =1; % 1 simulate a breakdown on all heaters on 5th for 6 hours 
fprintf('*** RMPC Scenario 3S')
outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);
BePlot(outdata,PlotParam)
