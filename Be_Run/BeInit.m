%% BeSim 
% Matlab toolbox for fast developlent, simulation and deployment of
% advanced building climate controllers 
% Model Predictive Control (MPC)
% functionality intended for automatic construction of controls and
% estimation for a given linear building model

clear ;
yalmip('clear');
% close all

addpath('../Be_Modeling/')
addpath('../Be_Disturbances/')
addpath('../Be_References/')
addpath('../Be_Estimation/')
addpath('../Be_Control/')
addpath('../Be_Simulation/')
addpath('../Be_Learn/')

%% Model: emulator + prediction

% =========== 1, choose building model =================
% select from a library of available models 
% buildingType = ModelIdentifier 
% ModelIdentifier for residential houses with radiators:   'Reno', 'Old', 'RenoLight'
% ModelIdentifier for office buildings with TABS:          'Infrax', 'HollandschHuys'
% ModelIdentifier for borehole:                            'Borehole'  - % TODO: missing disturbances precomputed file for borehole 
buildingType = 'Reno';  

% =========== 2, choose model order =================
ModelParam.Orders.range = [4, 7, 10, 15, 20, 30, 40, 100];    % suggested = model orders for 'Reno', 'Old', 'RenoLight'
% ModelParam.Orders.range = [100, 200, 600];                  % suggested model orders for 'Infrax', 'HollandschHuys'
ModelParam.Orders.choice = 'full'; % L2020 March 4 'full';                            % model order selection for prediction
ModelParam.off_free = 0; %L2020 -change to 0 and observer the difference,                                      % augmented model with unmeasured disturbances
ModelParam.reload = 0;                                        % if 1 reload ROM, if 0 load saved ROM

% =========== 4, choose model analysis =================
ModelParam.analyze.SimSteps = 2*672; % Number of simulation steps (Ts = 900 s),  672 = one week
ModelParam.analyze.openLoop.use = false;             %  open loop simulation   - TODO
ModelParam.analyze.openLoop.start = 1;              % starting day of the analysis
ModelParam.analyze.openLoop.end = 7;                % ending day of the analysis
ModelParam.analyze.nStepAhead.use = false;           % n-step ahead predicion error  - TODO
ModelParam.analyze.nStepAhead.steps = [1, 10, 40];  % x*Ts  
ModelParam.analyze.HSV = false;                      %  hankel singular values of ROM
ModelParam.analyze.frequency = false;                % frequency analysis - TODO

% =========== 4, construct model structue =================
model_Light = BeModel('RenoLight', ModelParam);      % construct a model object   
model_Reno = BeModel('Reno', ModelParam);      % construct a model object   
model_Old = BeModel('Old', ModelParam);      % construct a model object   

model_Light.pred.umax = model_Old.pred.umax;
model_Reno.pred.umax = model_Old.pred.umax;

model = model_Reno;      % construct a model for prediction   
model_plant = model_Reno; % construct a model for plant simulation 

%% Disturbacnes 
% ambient temperature, solar radiation, internal heat gains
DistParam.reload = 0;

dist = BeDist(model_Reno, DistParam);        % construct a disturbances object 

%% References 
% comfort constraints, price profiles
RefsParam.Price.variable = 0;       %1 =  variable price profile, 0 = fixed to 1

refs = BeRefs(model_Reno, dist, RefsParam);     % construct a references object  

%% Estimator 
EstimParam.SKF.use = 0;          % stationary KF
EstimParam.TVKF.use = 1;         % time varying KF
EstimParam.MHE.use = 0;          % moving horizon estimation via yalmip
EstimParam.MHE.Condensing = 1;   % state condensing 
EstimParam.use = 1;

estim = BeEstim(model_Reno, EstimParam);      % construct an estimator object  

%% Controller 
CtrlParam.use = 1;   % 0 for precomputed u,y    1 for closed loop control
CtrlParam.MPC.use = 1;%L2020 March 3 orignal = 1
CtrlParam.MPC.Condensing = 1;%L2020 March 3 orignal = 1
CtrlParam.LaserMPC.use = 0;
CtrlParam.LaserMPC.Condensing = 1;
CtrlParam.RBC.use = 0;
CtrlParam.PID.use = 0;
CtrlParam.MLagent.use = 0;
CtrlParam.RMPC.use=0;
CtrlParam.RMPC.Condensing = 1;
CtrlParam.ARMPC.use=0;%L2020 MUP USE % can be change to GSRMPC
CtrlParam.ARMPC.Condensing = 1;
CtrlParam.RMPCLMI.use=0; %L2020 MUP USE 

if (CtrlParam.RMPCLMI.use||CtrlParam.ARMPC.use) % because we need 3 model to create these controllers 
     ctrl = BeCtrl_1(model_Old,model_Reno,model_Light,CtrlParam); % first model should be the plant model for LMI 
 else
    ctrl = BeCtrl(model, CtrlParam);       % construct a controller object
end
%% Simulate
% SimParam.run.start = 11;
% SimParam.run.end = 17; 
SimParam.run.start =10;%S=200, W=10
SimParam.run.end = 16;%S=206, W=16 
SimParam.verbose = 1;
SimParam.flagSave = 0;
SimParam.comfortTol = 1e-1;
SimParam.emulate = 1;  % emulation or real measurements:  0 = measurements,  1 = emulation
SimParam.profile = 0;  % profiler function for CPU evaluation
SimParam.allDist =1;  % 1 = provide all the disturbance to the controller, 0= only Ambient Temperature will be provided to Controller
SimParam.breakdown =0;% 1 simulate a breakdown on all heaters on 5th for 6 hours 
SimParam.Uncertainty =1;% to simulate the additive uncertainty in the ambient temperature 
% %  simulation file with embedded plotting file

%outdata = BeSim(model, estim, ctrl, dist, refs, SimParam);

if CtrlParam.ARMPC.use
    %provide here the plant, the model to do gain-schedule strategy among them
    outdata = BeSim_3(model_plant,model_Old,model_Reno,model_Light, estim, ctrl, dist, refs, SimParam);
else
    outdata = BeSim_2(model_plant,model, estim, ctrl, dist, refs, SimParam);% if there is no mis-match betweent the model and plant, you can set them both the same
end

%% Diagnose the MPC problem via Yalmip optimize
DiagnoseParam.diagnoseFlag = 0; % old value :true; %L2020-March 2, 
DiagnoseParam.Duals.plotCheck = 0;
DiagnoseParam.Reduce.lincols.use = 1;
DiagnoseParam.Reduce.PCA.use = 1;
DiagnoseParam.Reduce.PCA.normalize = 0;             % normalize constraints based on types
DiagnoseParam.Reduce.PCA.component = 0.999999999;   % principal component weight threshold
DiagnoseParam.Reduce.PCA.feature = 0.999999999;     % PCA features weight threshold
if DiagnoseParam.diagnoseFlag
    % solve single instance of the MPC problem via Yalmip optimize
    [diagnostics, con, obj, outdata.con_info] = BeMPC_DualCheck(outdata, model, DiagnoseParam);
end

%% Plot Results
PlotParam.flagPlot = true;          % plot 0 - no 1 - yes
PlotParam.plotStates = 0;        % plot states
PlotParam.plotStates3D = 0;      % ribbon plot states
PlotParam.plotDist = 0;          % plot disturbances
PlotParam.plotDist3D = 0;        % ribbon plot disturbances
PlotParam.plotEstim = 0;         % plot estimation
PlotParam.plotEstim3D = 0;       % ribbon plot estimation
PlotParam.plotCtrl = 1;          % plot control
PlotParam.plotPrice = 0;          % plot price signal
if DiagnoseParam.diagnoseFlag
    PlotParam.plotPrimalDual = 1;          % plot primal and dual varibles
    PlotParam.plotPrimalDual3D = 1;        % ribbon plot primal and dual varibles
    PlotParam.plotDualActive = 1;     % activation of the dual varibles
    PlotParam.plotPCA_Dual = 1;        % principal components of PCA reduced dual variables   
    PlotParam.plotActiveSet = 1;         % plot active sets
else
    PlotParam.plotPrimalDual = 0;          % plot primal and dual varibles
    PlotParam.plotPrimalDual3D = 0;        % ribbon plot primal and dual varibles
    PlotParam.plotDualActive = 0;     % activation of the dual varibles
    PlotParam.plotPCA_Dual = 0;        % principal components of PCA reduced dual variables 
    PlotParam.plotActiveSet = 0;     % plot active sets
end
% PlotParam.Transitions = 1;      % pot dynamic transitions of Ax matrix
% PlotParam.reduced = 0;   %  reduced paper plots formats 0 - no 1 - yes
% PlotParam.zone = 2;     % choose zone if reduced
% PlotParam.only_zone = 0;    %  plot only zone temperatures 0 - no 1 - yes  

if PlotParam.flagPlot
    BePlot(outdata,PlotParam)
end

%% Save Results
SaveParam.path = ['../Data/Simulations/',buildingType]; % savepath
SaveParam.save = true;                     % save or not
SaveParam.data.states = true;              % X      
SaveParam.data.outputs = true;              % Y    
SaveParam.data.inputs = true;               % U   
SaveParam.data.disturbances = true;         % D 
SaveParam.data.references = true;          % WA, WB 
SaveParam.solver.objective = true;          % objective values of the QP optimization problem
SaveParam.solver.duals = true;              % dual variables of the QP optimization problem
SaveParam.solver.primals = true;           % primal variables of the QP optimization problem
SaveParam.solver.PCA_duals = true;            % save principal components of PCA reduced dual variables 
SaveParam.data.ActiveSets = true;           % save active sets = uniqe combinations of active constraints
SaveParam.solver.SolverTime = true;         % solvertime
SaveParam.solver.iters = true;              % solver iterations
SaveParam.solver.specifics = false;         % solver specific information

if SaveParam.save && DiagnoseParam.diagnoseFlag
    BeSave(outdata,SaveParam)
end







 