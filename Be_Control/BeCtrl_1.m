function controller = BeCtrl_1(model_O,model_R,model_L, CtrlParam)

if nargin == 0
   buildingType = 'Infrax';  
   ModelOrders.range = [100, 200, 600]; % add any reduced order model you wish to have
   ModelOrders.choice = 200;            % insert model order or 'full' for full order SSM 
   ModelOrders.off_free = 0;            %  augmented model
   reload = 0;
%    construct the model
   model = BeModel(buildingType, ModelOrders, reload); 
end
if nargin < 2
   CtrlParam.use = 1;
%    CtrlParam.precomputed = 1;
   CtrlParam.MPC.use = 0;
   CtrlParam.MPC.Condensing = 1;
   CtrlParam.LaserMPC.use = 0;
   CtrlParam.LaserMPC.Condensing = 1;
   CtrlParam.RBC.use = 0;
   CtrlParam.PID.use = 0;
   CtrlParam.MLagent.use = 0;
end
controller.use = CtrlParam.use;
% controller.precomputed.use = CtrlParam.precomputed;
controller.MPC.use =    CtrlParam.MPC.use;
controller.MPC.Condensing =    CtrlParam.MPC.Condensing;

controller.LaserMPC.use =    CtrlParam.LaserMPC.use;
controller.LaserMPC.Condensing =    CtrlParam.LaserMPC.Condensing;

controller.RBC.use =    CtrlParam.RBC.use;
controller.PID.use =    CtrlParam.PID.use;
controller.MLagent.use =    CtrlParam.MLagent.use;

controller.RMPC.use=    CtrlParam.RMPC.use; % L2020 RMPC MUP Use 
controller.RMPC.Condensing =    CtrlParam.RMPC.Condensing;

controller.RMPCLMI.use=    CtrlParam.RMPCLMI.use; % L2020 RMPC MUP Use 

fprintf('\n------------------ Controller -----------------------\n');
% add elseif controller.RMPCMUP.use then prepare calling MUP 

    
if CtrlParam.RMPCLMI.use  
    
    fprintf('*** Create LMI RMPC controller ... \n')
   
if  strcmp(model_O.buildingType,'HollandschHuys')    
     % horizons
    controller.RMPCLMI.N = 32;
    controller.RMPCLMI.Nc = 32;
    controller.RMPCLMI.Nrp = 32;
    controller.RMPCLMI.Ndp = 32;
    % weight diagonal matrices 
    controller.RMPCLMI.Qsb = 1e10*eye(model_O.pred.ny);
    controller.RMPCLMI.Qsa = 1e10*eye(model_O.pred.ny);
    controller.RMPCLMI.Qu = 1e0*eye(model_O.pred.nu);
else 
     % horizons
    controller.RMPCLMI.N = 22;
    controller.RMPCLMI.Nc = 22;
    controller.RMPCLMI.Nrp = 22;
    controller.RMPCLMI.Ndp = 22;
    % weight diagonal matrices 
    controller.RMPCLMI.Qsb = 1e10*eye(model_O.pred.ny);%10
    controller.RMPCLMI.Qsa = 1e4*eye(model_O.pred.ny);
    controller.RMPCLMI.Qw = 1e0*eye(model_O.pred.nx);%-3%1e6 1e5*eye(nx)
    controller.RMPCLMI.Qy = 1e0*eye(model_O.pred.nu);%1e1
end   
    %  MPC optimizer synthesis   
    [controller.RMPCLMI.optimizer, controller.RMPCLMI.constraints_info] = BeRMPCLMI(model_O,model_R,model_L);
    fprintf('*** Done.\n')
 
    
    
end





end