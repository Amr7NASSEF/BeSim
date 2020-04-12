function controller = BeCtrl(model, CtrlParam)

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

% controller parameters
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
% %  % input constraints  [W]
% % controller.umax = ; % 
% % controller.umin = ; % 
% if strcmp(model.buildingType,'Reno')
%     controller.umax = [1680, 685, 154, 1000, 320, 232]'; 
% elseif strcmp(model.buildingType,'Old')
%     controller.umax = [2940, 960, 300, 1400, 460, 253]';
% elseif strcmp(model.buildingType,'RenoLight')
%     controller.umax = [1680, 685, 154, 1000, 320, 232]'/2; 
% % else
% %     disp('no input constraints');
% end

fprintf('\n------------------ Controller -----------------------\n');
% add elseif controller.RMPCMUP.use then prepare calling MUP 

if not(controller.use)    % precomputed inputs and outputs or real measurements
    fprintf('*** Load pre-computed controls ... \n')
    path = ['../buildings/', model.buildingType];  
    load([path '/preComputed_matlab/preComputedControls.mat']);
    controller.precomputed.U = U;  
    controller.precomputed.Y = Y;    
    fprintf('*** Done.\n') 
    
        %    CTRL DESIGN 
% RBC, MPC, PID, ML, etc
    
    
elseif CtrlParam.MPC.use  
    fprintf('*** Create MPC controller ... \n')
   
if  strcmp(model.buildingType,'HollandschHuys')    
     % horizons
    controller.MPC.N = 32;
    controller.MPC.Nc = 32;
    controller.MPC.Nrp = 32;
    controller.MPC.Ndp = 32;
    % weight diagonal matrices 
    controller.MPC.Qsb = 1e10*eye(model.pred.ny);
    controller.MPC.Qsa = 1e10*eye(model.pred.ny);
    controller.MPC.Qu = 1e0*eye(model.pred.nu);
else 
     % horizons
    controller.MPC.N = 22;
    controller.MPC.Nc = 22;
    controller.MPC.Nrp = 22;
    controller.MPC.Ndp = 22;
    % weight diagonal matrices 
    controller.MPC.Qsb = 1e8*eye(model.pred.ny);
    controller.MPC.Qsa = 1e8*eye(model.pred.ny);
    controller.MPC.Qu = 1e0*eye(model.pred.nu);
end   
    %  MPC optimizer synthesis   
    [controller.MPC.optimizer, controller.MPC.constraints_info] = BeMPCdesign(model, controller.MPC);
    fprintf('*** Done.\n')
    
elseif CtrlParam.LaserMPC.use  
    fprintf('*** Create LaserMPC controller ... \n')

   
if  strcmp(model.buildingType,'HollandschHuys')    
     % horizons
    controller.LaserMPC.N = 32;
    controller.LaserMPC.Nc = 32;
    controller.LaserMPC.Nrp = 32;
    controller.LaserMPC.Ndp = 32;
    % weight diagonal matrices 
    controller.LaserMPC.Qsb = 1e10*eye(model.pred.ny);
    controller.LaserMPC.Qsa = 1e10*eye(model.pred.ny);
    controller.LaserMPC.Qu = 1e0*eye(model.pred.nu);
else 
     % horizons
    controller.LaserMPC.N = 22;
    controller.LaserMPC.Nc = 22;
    controller.LaserMPC.Nrp = 22;
    controller.LaserMPC.Ndp = 22;
    % weight diagonal matrices 
    controller.LaserMPC.Qsb = 1e8*eye(model.pred.ny);
    controller.LaserMPC.Qsa = 1e8*eye(model.pred.ny);
    controller.LaserMPC.Qu = 1e0*eye(model.pred.nu);
end   
    controller.LaserMPC.RelaxConTagsDim = 3;  % this will be a variable for a generic MPC formulation
    %  MPC optimizer synthesis   
    [controller.LaserMPC.optimizer, controller.LaserMPC.constraints_info] = LaserMPCdesign(model, controller.LaserMPC);
       
    fprintf('*** Done.\n')    
%% March 2020
elseif CtrlParam.RMPC.use  
    
    fprintf('*** Create RMPC controller ... \n')
   
if  strcmp(model.buildingType,'HollandschHuys')    
     % horizons
    controller.RMPC.N = 32;
    controller.RMPC.Nc = 32;
    controller.RMPC.Nrp = 32;
    controller.RMPC.Ndp = 32;
    % weight diagonal matrices 
    controller.RMPC.Qsb = 1e10*eye(model.pred.ny);
    controller.RMPC.Qsa = 1e10*eye(model.pred.ny);
    controller.RMPC.Qu = 1e0*eye(model.pred.nu);
else 
     % horizons
    controller.RMPC.N = 22;%3;%22;
    controller.RMPC.Nc = 22;%22;
    controller.RMPC.Nrp = 22;%22;
    controller.RMPC.Ndp = 22;%22;
    % weight diagonal matrices 
    controller.RMPC.Qsb = 1e8*eye(model.pred.ny);
    controller.RMPC.Qsa = 1e8*eye(model.pred.ny);
    controller.RMPC.Qu  = 1e2*eye(model.pred.nu);
    
end   
    %  MPC optimizer synthesis   
    [controller.RMPC.optimizer, controller.RMPC.constraints_info] = BeRMPCMINMAX(model, controller.RMPC);
    fprintf('*** Done.\n')
    
    
elseif CtrlParam.RMPCLMI.use  
    
    fprintf('*** Create LMI RMPC controller ... \n')
    
    model_Light = BeModel('RenoLight', ModelParam);      % construct a model object   
    model_Reno = BeModel('Reno', ModelParam);      % construct a model object   
    model_Old = BeModel('Old', ModelParam);      % construct a model object   
    
   
if  strcmp(model.buildingType,'HollandschHuys')    
     % horizons
    controller.RMPCLMI.N = 32;
    controller.RMPCLMI.Nc = 32;
    controller.RMPCLMI.Nrp = 32;
    controller.RMPCLMI.Ndp = 32;
    % weight diagonal matrices 
    controller.RMPCLMI.Qsb = 1e10*eye(model.pred.ny);
    controller.RMPCLMI.Qsa = 1e10*eye(model.pred.ny);
    controller.RMPCLMI.Qu = 1e0*eye(model.pred.nu);
else 
     % horizons
    controller.RMPCLMI.N = 22;
    controller.RMPCLMI.Nc = 22;
    controller.RMPCLMI.Nrp = 22;
    controller.RMPCLMI.Ndp = 22;
    % weight diagonal matrices 
    controller.RMPCLMI.Qsb = 1e2*eye(model.pred.ny);%10
    controller.RMPCLMI.Qsa = 1e4*eye(model.pred.ny);
    controller.RMPCLMI.Qw = 1e6*eye(model.pred.nx);%-3%1e6 1e5*eye(nx)
    controller.RMPCLMI.Qy = 1e1*eye(model.pred.nu);%1e1
end   
    %  MPC optimizer synthesis   
    [controller.RMPCLMI.optimizer, controller.RMPCLMI.constraints_info] = BeRMPCLMIdesign(model, controller.RMPCLMI);
    fprintf('*** Done.\n')
    
    
elseif CtrlParam.PID.use  
    fprintf('*** Create PID controller ... \n')
%     TODO
    
    
    fprintf('*** Done.\n')

    
elseif CtrlParam.RBC.use      %% RBC heat curve controller
    fprintf('*** Create RBC controller ... \n')
%     TODO
    controller.RBC.w = 0.5; %  on off thermostat width of the switching zone zone
    controller.RBC.zone = 2; % zone = choose location of the on-off thermostat (output)   
    
    fprintf('*** Done.\n')

    
% elseif CtrlParam.ML.use  
    
    
end





end