function controller = BeCtrl_1(model_11,model_22,model_33, CtrlParam)
nv=nargin-1;
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
controller.AMPC.use =    CtrlParam.AMPC.use;
controller.AMPC.Condensing =    CtrlParam.AMPC.Condensing;
controller.MHEParams.Condensing= CtrlParam.AMPC.MHE.Condensing;

controller.LaserMPC.use =    CtrlParam.LaserMPC.use;
controller.LaserMPC.Condensing =    CtrlParam.LaserMPC.Condensing;
controller.RBC.use =    CtrlParam.RBC.use;
controller.PID.use =    CtrlParam.PID.use;
controller.MLagent.use =    CtrlParam.MLagent.use;
controller.RMPC.use=    CtrlParam.RMPC.use; % L2020 RMPC MUP Use 
controller.RMPC.Condensing =    CtrlParam.RMPC.Condensing;
controller.ARMPC.use=    CtrlParam.ARMPC.use; % L2020 RMPC MUP Use 
controller.ARMPC.Condensing =    CtrlParam.ARMPC.Condensing;
controller.RMPCLMI.use=    CtrlParam.RMPCLMI.use; % L2020 RMPC MUP Use 

fprintf('\n------------------ Controller -----------------------\n');
% add elseif controller.RMPCMUP.use then prepare calling MUP 
if CtrlParam.ARMPC.use  
    
    fprintf('*** Create ARMPC controller ... \n')

    
if  strcmp(model_11.buildingType,'HollandschHuys')    
     % horizons
    controller.ARMPC.N = 32;
    controller.ARMPC.Nc = 32;
    controller.ARMPC.Nrp = 32;
    controller.ARMPC.Ndp = 32;
    % weight diagonal matrices 
    controller.ARMPC.Qsb = 1e10*eye(model_11.pred.ny);
    controller.ARMPC.Qsa = 1e10*eye(model_11.pred.ny);
    controller.ARMPC.Qu = 1e0*eye(model_11.pred.nu);
else 
     % horizons
    controller.ARMPC.N = 22;%3;%22;
    controller.ARMPC.Nc = 22;%22;
    controller.ARMPC.Nrp = 22;%22;
    controller.ARMPC.Ndp = 22;%22;
    % weight diagonal matrices 
    controller.ARMPC.Qsb = 1e8*eye(model_11.pred.ny);
    controller.ARMPC.Qsa = 1e8*eye(model_11.pred.ny);
    controller.ARMPC.Qu  = 1e2*eye(model_11.pred.nu);
    
end   
for i=1:nv
    eval(['[controller.ARMPC.optimizer' num2str(i) ', controller.ARMPC.constraints_info' num2str(i) ']' '= BeRMPCMINMAX(model_' num2str(i) num2str(i) ', controller.ARMPC);'])
    fprintf(['***Controller_' num2str(i) '_Done.\n'])    
  
end
end

    
if CtrlParam.RMPCLMI.use  
    
    fprintf('*** Create LMI RMPC controller ... \n')
   
if  strcmp(model_11.buildingType,'HollandschHuys')    
     % horizons
    controller.RMPCLMI.N = 32;
    controller.RMPCLMI.Nc = 32;
    controller.RMPCLMI.Nrp = 32;
    controller.RMPCLMI.Ndp = 32;
    % weight diagonal matrices 
    controller.RMPCLMI.Qsb = 1e10*eye(model_11.pred.ny);
    controller.RMPCLMI.Qsa = 1e10*eye(model_11.pred.ny);
    controller.RMPCLMI.Qu = 1e0*eye(model_11.pred.nu);
else 
     % horizons
    controller.RMPCLMI.N = 22;
    controller.RMPCLMI.Nc = 22;
    controller.RMPCLMI.Nrp = 22;
    controller.RMPCLMI.Ndp = 22;
    % weight diagonal matrices 
    controller.RMPCLMI.Qsb = 1e10*eye(model_11.pred.ny);%10
    controller.RMPCLMI.Qsa = 1e4*eye(model_11.pred.ny);
    controller.RMPCLMI.Qw = 1e0*eye(model_11.pred.nx);%-3%1e6 1e5*eye(nx)
    controller.RMPCLMI.Qy = 1e0*eye(model_11.pred.nu);%1e1
end   
    %  MPC optimizer synthesis   
    [controller.RMPCLMI.optimizer, controller.RMPCLMI.constraints_info] = BeRMPCLMI(model_11,model_22,model_33);
    fprintf('*** Done.\n')
 
    
    
end





end