function outdata = BeSim_3(model,model_1,model_2,model_3 ,estim, ctrl, dist, refs, SimParam)

if nargin == 0  % model
   buildingType = 'Infrax';  
   ModelOrders.range = [100, 200, 600]; % add any reduced order model you wish to have
%    ModelOrders.choice = 200;            % insert model order or 'full' for full order SSM 
   ModelOrders.choice = 100;            % insert model order or 'full' for full order SSM 
   ModelOrders.off_free = 0;            %  augmented model
   reload = 0;
   model = BeModel(buildingType, ModelOrders, reload);     %  construct the model
end
if nargin < 2   % estimator
   EstimParam.LOPP.use = 0;      %  Luenberger observer via pole placement 
   EstimParam.SKF.use = 0;    % stationary KF
   EstimParam.TVKF.use = 1;   % time varying KF
   EstimParam.MHE.use = 0;   % moving horizon estimation via yalmip
   EstimParam.MHE.Condensing = 1;   % moving horizon estimation via yalmip
   estim = BeEstim(model, EstimParam);
end
if nargin < 3   % controller
   CtrlParam.precomputed = 1;
   CtrlParam.MPC.use = 0;
   CtrlParam.MPC.Condensing = 1;
   CtrlParam.LaserMPC.use = 0;
   CtrlParam.LaserMPC.Condensing = 1;
   CtrlParam.RBC.use = 0;
   CtrlParam.PID.use = 0;
   ctrl = BeCtrl(model, CtrlParam);
end
if nargin < 4  % disturbances
   dist = BeDist(model.buildingType, model.reload);
end
if nargin < 5   % references
   refs = BeRefs();      % TODO FIX to be general
end
if nargin < 6   % simulation parameters
    SimParam.run.start = 1;     % starting day
    SimParam.run.end = 7;       % finishing day
    SimParam.verbose = 1;
    SimParam.flagSave = 0;
    SimParam.comfortTol = 1e-1;
    SimParam.profile = 0; 
    SimParam.emulate = 1;  % emulation or real measurements:  0 = measurements,  1 = emulation
end
% matlab profiler function for CPU evaluation
if SimParam.profile
    profile on 
end

%% Simulation setup   
fprintf('\n------------------ Simulation Setup -----------------\n');

%fprintf('*** Building Type Prediction = %s\n' , model_1.buildingType);
fprintf('*** Building Type Plant = %s\n' , model.buildingType);
fprintf('*** r order = %d, \n',   size(model_1.pred.Ad,1))
fprintf('*** Start day = %d , End day = %d \n', SimParam.run.start, SimParam.run.end);

% Simulation steps   
% starting and finishing  second:  24h = 86400 sec
SimStart_sec = (SimParam.run.start-1)*86400;
SimStop_sec = (SimParam.run.end)*86400;
% starting and finishing  step for simulation loop - MPC
SimStart = floor(SimStart_sec/model.plant.Ts)+1;
SimStop = ceil(SimStop_sec/model.plant.Ts);
% number of simulation steps for MPC
Nsim = length(SimStart:SimStop);

%% Initial values  

% if not(exist('ctrl.MLagent'))
%     ctrl.MLagent.use = 0;
% end

% preview setup
if ctrl.MPC.use 
        N = ctrl.MPC.N;
        Nrp = ctrl.MPC.Nrp;   
elseif ctrl.LaserMPC.use
        N = ctrl.LaserMPC.N;
        Nrp = ctrl.LaserMPC.Nrp;           
elseif ctrl.MLagent.use
        N = ctrl.MLagent.numDelays;
        Nrp = ctrl.MLagent.numDelays;
%% March 2020
elseif ctrl.RMPC.use
        N = ctrl.RMPC.N;
        Nrp = ctrl.RMPC.Nrp;
        
elseif ctrl.ARMPC.use
    N = ctrl.ARMPC.N;
    Nrp = ctrl.ARMPC.Nrp;

elseif ctrl.RMPCLMI.use
        N = ctrl.RMPCLMI.N;
        Nrp = ctrl.RMPCLMI.Nrp;
else
        N = 0;
        Nrp = 0;
end

% state disturbacne trajectories
X = zeros(model.plant.nx,Nsim+1);
Dp = dist.d(SimStart:SimStop+N,:)';% disturbance injected into the plant

if SimParam.allDist ==1
    D = dist.d(SimStart:SimStop+N,:)';% disturbance injected into the controller
else
    D = dist.d(SimStart:SimStop+N,:)'*0;
    D(41,:) = dist.d(SimStart:SimStop+N,41)';  
end



% diagnostics
StepTime = zeros(1,Nsim); 
MPCTime = zeros(1,Nsim); 

% realisrtic states initialization for particular models
if  strcmp(model.buildingType,'HollandschHuys')
    % building parameters
    path = ['../buildings/', model.buildingType];
    load([path '/preComputed_matlab/X_initialization.mat'],'x_init');
%     load('X_initialization.mat','x_init')
	X(:,1) = x_init;
end

% % arbitraty offset for state initialization
% X(:,1) = X(:,1) + ones(model.plant.nx,1)*model.analyze.XinitOffset;

if  not(ctrl.use)  % precomputed inputs and outputs
    U = ctrl.precomputed.U(:,SimStart:SimStop);
%     Y = ctrl.precomputed.Y(:,SimStart:SimStop)+273.15;   % infrax - signal in deg C
    Y = ctrl.precomputed.Y(:,SimStart:SimStop);    
    uopt = 0*U(:,1); % initialize controls
    
else   % initialize matrices for closed loop control simulations
    Y = zeros(model.plant.ny,Nsim)+model.plant.Fd*1;
    U = zeros(model.plant.nu,Nsim);
    
    uopt = U(:,1); % initialize controls

    % ------ references ------
    if  strcmp(model.buildingType,'Reno') ||  strcmp(model.buildingType,'Old') ||  strcmp(model.buildingType,'RenoLight')
         R = refs.R(SimStart:SimStop,:)';
    end
    % above and below threshold comfort zone
    wa = refs.wa(SimStart:SimStop+N,:)';
    wb = refs.wb(SimStart:SimStop+N,:)';
    % ------ PMV zone ------ 
    PMVub = refs.PMVub(SimStart:SimStop);
    PMVlb = refs.PMVlb(SimStart:SimStop);
    % ------ energy price ------
    Price = refs.Price(SimStart:SimStop+N,:)';

    if ctrl.RBC.use
        % supply water temperature
        TSup = refs.TSup(SimStart:SimStop,:)';
        heat = 0;
    end
end

if estim.use 
    %     estmator vector inits
    if   ctrl.ARMPC.use
        Xp_1 = zeros(model_1.pred.nx,Nsim+1);
        Xp_2 = zeros(model_2.pred.nx,Nsim+1);
        Xp_3 = zeros(model_3.pred.nx,Nsim+1);
    else
    Xp = zeros(model_1.pred.nx,Nsim+1);
    end
    Xe = zeros(model_1.pred.nx,Nsim);
   
    Ye = zeros(model_1.pred.ny,Nsim);
    Yp = zeros(model_1.pred.ny,Nsim);
%     Ye = 295.15*ones(model_pred1.pred.ny,Nsim);
%     Yp = 295.15*ones(model_pred1.pred.ny,Nsim);
%     Ye = 21.15*ones(model_pred1.pred.ny,Nsim);
%     Yp = 21.15*ones(model_pred1.pred.ny,Nsim);
    
    % current estim states
     
    if   ctrl.ARMPC.use
        xp_1 = Xp_1(:, 1);
        xp_2 = Xp_2(:, 1);
        xp_3 = Xp_3(:, 1);
    else
     xp = Xp(:, 1); 
    end
    
    xe = Xe(:, 1);
    ye = Ye(:, 1);  
    yp = Yp(:, 1); 
    
    if estim.TVKF.use
        EstimGain = cell(1,Nsim);
        ErrorCovar = cell(1,Nsim); 
    end

    if estim.MHE.use
        OBJ_MHE = zeros(1,Nsim);
        We = zeros(model_1.pred.nx,Nsim);
        Ve = zeros(model_1.pred.ny,Nsim);    
    end   
end


% initialize MPC 
if ctrl.MPC.use
%     initialize MPC diagnostics vectors
        if model.plant.nd == 0  %  no disturbnances option
             [opt_out, feasible, info1, info2, info3, info4] =  ctrl.MPC.optimizer{{X(:,1), wa(:,1:Nrp), wb(:,1:Nrp), Price(:,1:N)}}; % optimizer with estimated states
        else
             [opt_out, feasible, info1, info2, info3, info4] =  ctrl.MPC.optimizer{{Xp(:, 1), D(:,1:N), wa(:,1:Nrp), wb(:,1:Nrp), Price(:,1:N)}}; % optimizer with estimated states          
       
        end
 
end

% initialize Laser MPC 
if ctrl.LaserMPC.use 
%     initialize active set flags, 0 = active constrains, 1 = inactive constraint
        Delta = zeros(ctrl.LaserMPC.RelaxConTagsDim,Nsim+N);
%     initialize Laser MPC diagnostics vectors
        if model.plant.nd == 0  %  no disturbnances option
             [opt_out, feasible, info1, info2, info3, info4] =  ctrl.LaserMPC.optimizer{{X(:,1), wa(:,1:Nrp), wb(:,1:Nrp), Price(:,1:N), Delta(:,1:N)}}; % optimizer with estimated states
        else
             [opt_out, feasible, info1, info2, info3, info4] =  ctrl.LaserMPC.optimizer{{Xp(:, 1), D(:,1:N), wa(:,1:Nrp), wb(:,1:Nrp), Price(:,1:N), Delta(:,1:N)}}; % optimizer with estimated states          
        end
end

if ctrl.RMPC.use
    
%     initialize MPC diagnostics vectors
        if model.plant.nd == 0  %  no disturbnances option
             
             [opt_out, feasible, info1, info2, info3, info4] =  ctrl.RMPC.optimizer{{X(:,1), wa(:,1:Nrp), wb(:,1:Nrp), Price(:,1:N)}}; % optimizer with estimated states
        else
             [opt_out, feasible, info1, info2, info3, info4] =  ctrl.RMPC.optimizer{{Xp(:, 1), D(:,1:N), wa(:,1:Nrp), wb(:,1:Nrp), Price(:,1:N)}}; % optimizer with estimated states          
        end
 
end

if ctrl.ARMPC.use
%     initialize MPC diagnostics vectors
        if model.plant.nd == 0  %  no disturbnances option
             
             [opt_out_1, feasible_1, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer1{{X(:,1), wa(:,1:Nrp), wb(:,1:Nrp), Price(:,1:N)}}; % optimizer with estimated states
             [opt_out_2, feasible_2, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer2{{X(:,1), wa(:,1:Nrp), wb(:,1:Nrp), Price(:,1:N)}}; % optimizer with estimated states
             [opt_out_3, feasible_3, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer3{{X(:,1), wa(:,1:Nrp), wb(:,1:Nrp), Price(:,1:N)}}; % optimizer with estimated states

        else
             
            [opt_out_1, feasible_1, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer1{{Xp_1(:, 1), D(:,1:N), wa(:,1:Nrp), wb(:,1:Nrp), Price(:,1:N)}}; % optimizer with estimated states          
            [opt_out_2, feasible_2, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer2{{Xp_2(:, 1), D(:,1:N), wa(:,1:Nrp), wb(:,1:Nrp), Price(:,1:N)}}; % optimizer with estimated states          
            [opt_out_3, feasible_3, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer3{{Xp_3(:, 1), D(:,1:N), wa(:,1:Nrp), wb(:,1:Nrp), Price(:,1:N)}}; % optimizer with estimated states          

        end
        choix=3;
        
 
end



if ctrl.RMPCLMI.use
    
%L2020
%Finding steady state input Us and steady state states Xs
%accoridng to Reference R(k) = CXs(k), Xs(k) =A Xs + B Us
% Xss = AXss +BUss +E*d + Gd
% yss=Ref = CXss + DUss + Fd
% M^-1 * [Xss;USS] = [-(E*d + Gd);(R-Fd)]
%first we create M^-1 which is a matrix [(A-I_nx) B ; C , D ]% if Nu!= Ny
% D should work to make the matrix M square such that we have the inverse     
    
    
u = zeros(model.pred.nu,1);

M   = inv([[model.pred.Ad - eye(model.pred.nx)], model.pred.Bd; model.pred.Cd, model.pred.Dd]);
M1  = M(1:model.pred.nx,:);
M2  = M(model.pred.nx+1:end,:);    
Xss = M1 * [-(model.pred.Ed*D(:,1) + model.pred.Gd); [R(:,1) - model.pred.Fd]];
Uss = M2 * [-(model.pred.Ed*D(:,1) + model.pred.Gd); [R(:,1) - model.pred.Fd]];
 
        if model.plant.nd == 0  %  no disturbnances option
          Xer = X(:,1) - Xss;% X(:,1): zeros   
        else
          Xer = Xp(:,1) - Xss;% X(:,1): zeros 
        end

Ue = u - Uss;% 

ymaxred = wa(:,1);% L4 maybe use wb

umax = model.pred.umax - Uss;
ymax = ymaxred - (model.pred.Cd*Xss + model.pred.Dd*Uss + model.pred.Fd); 



      %     initialize LMI MPC diagnostics vectors
             [solutions, feasible, info1, info2, info3, info4] =  ctrl.RMPCLMI.optimizer{{Xer,umax,ymax}}; % optimizer with estimated states
             
             Ynew = solutions{1};
             Qnew = solutions{2};
             gamma = solutions{3};
             F = Ynew*inv(Qnew);
             
             u = Uss + F*Xer;
             opt_out{1} = u;
end
    

if ctrl.MPC.use || ctrl.LaserMPC.use || ctrl.RMPC.use || ctrl.RMPCLMI.use
    OBJ =  zeros(1,Nsim);            %  MPC objective function
    %constr_nr = sum(ctrl.MPC.constraints_info.i_length);  % total number of constraints % L2020 RMPC instead of MPC
    DUALS = zeros(length(info4.Dual),Nsim);  %  MPC dual variables for constraints
    PRIMALS = zeros(length(info4.Primal) ,Nsim); 
    
    ITERS = nan(1,Nsim);  % number of iterations of the solver
    %INEQLIN = zeros(length(info4.solveroutput.lambda.ineqlin) ,Nsim);
    %EQLIN = zeros(length(info4.solveroutput.lambda.eqlin) ,Nsim);  
end


%     violation vectors
Viol = zeros(model.plant.ny,Nsim); 
AboveViol = zeros(model.plant.ny,Nsim);
BelowViol = zeros(model.plant.ny,Nsim); 
%     PMV vectors
PMV = zeros(model.plant.ny,Nsim); 
PMVViol =  zeros(model.plant.ny,Nsim); 
PMVAboveViol =  zeros(model.plant.ny,Nsim); 
PMVBelowViol =  zeros(model.plant.ny,Nsim); 



%% ------ MAIN simulation loop ------
    % % ------ Verbose ------
fprintf('\n---------------- Simulation Running -----------------');
% % initiation clearing string 
reverseStr = '';

start_t = clock;
%W = mean(dist.d(:,41)) - normrnd(mean(dist.d(1:Nsim,41)),0.5*var(dist.d(1:Nsim,41)),1,Nsim);
%W = Uncertainty(dist.d,Nsim,model.plant.Ts);
%mean(D(41,1:Nsim)) - normrnd(mean(D(41,1:Nsim)),0.5*var(D(41,1:Nsim)),1,Nsim);


load(['Uncertain60copy.mat']);


%W =  normrnd(mean(dist.d(:,41)),var(dist.d(:,41)),1,Nsim);
% tt=(1:Nsim)*model.plant.Ts/3600/24;
% plot(tt,W); 
% title('Ambient Temperature Disturbance');
% ylabel('Temp [K]')%[^{\circ}C]'
% xlabel('time [days]')%
WW =zeros(44,1);

for k = 1:Nsim
    
%     current states, inputs, outputs and disturnances
    x0 = X(:,k);         % current sim states    - initialized to 0 at k = 1
    d0 = Dp(:,k);         % current disturbances  - from measurements     
    
%     TODO: implement 3 cases: 
% 1, plant simulation and control - all computed                 - DONE
% 2, measured (fixed) u and y                                    - DONE
% 3, measured (fixed) y - computed control and estimation        - TODO
    
%%  CONTROL  
% TODO standalone functions       
    if ctrl.use   
        if ctrl.RBC.use
            %  heat curve control 
            [uopt, heat] = BeRBC(yn,R(:,k),heat,TSup(k),ctrl);
%             TODO - automatic tuning general case         
            
        elseif ctrl.PID.use
%             TODO - implementation
            
        elseif ctrl.MPC.use || ctrl.LaserMPC.use  || ctrl.RMPC.use || ctrl.ARMPC.use || ctrl.RMPCLMI.use    
            
            if ctrl.MPC.use
    %             TODO: predictions as standalone function
                % preview of disturbance signals on the prediction horizon
                Dpreview = D(:, k:k+(ctrl.MPC.Ndp-1));   
                % preview of thresholds on the prediction horizon - Dynamic comfort zone
                wa_prev = wa(:, k:k+(ctrl.MPC.Nrp-1));
                wb_prev = wb(:, k:k+(ctrl.MPC.Nrp-1));
                % preview of the price signal
                Price_prev = Price(:, k:k+(ctrl.MPC.Nrp-1));
                if estim.use   % estimated states
                    if model.plant.nd == 0  %  no disturbnances option
                         [opt_out, feasible, info1, info2, info3, info4] =  ctrl.MPC.optimizer{{xp, wa_prev, wb_prev, Price_prev}}; % optimizer with estimated states
                    else
                         [opt_out, feasible, info1, info2, info3, info4] =  ctrl.MPC.optimizer{{xp, Dpreview, wa_prev, wb_prev, Price_prev}}; % optimizer with estimated states
                    end
                else    % perfect state update
                    if model.plant.nd == 0  %  no disturbnances option
                         [opt_out, feasible, info1, info2, info3, info4] =  ctrl.MPC.optimizer{{x0, wa_prev, wb_prev, Price_prev}}; % optimizer with estimated states
                    else
                         [opt_out, feasible, info1, info2, info3, info4] =  ctrl.MPC.optimizer{{x0, Dpreview, wa_prev, wb_prev, Price_prev}}; % optimizer with measured states  
                    end                 
                end

                MPC_options = ctrl.MPC.optimizer.options;
                
            elseif ctrl.LaserMPC.use   
                % preview of disturbance signals on the prediction horizon
                Dpreview = D(:, k:k+(ctrl.LaserMPC.Ndp-1));   
                % preview of thresholds on the prediction horizon - Dynamic comfort zone
                wa_prev = wa(:, k:k+(ctrl.LaserMPC.Nrp-1));
                wb_prev = wb(:, k:k+(ctrl.LaserMPC.Nrp-1));
                % preview of the price signal
                Price_prev = Price(:, k:k+(ctrl.LaserMPC.Nrp-1));
                % preview of estimated active sets - TODO: add ML model
                % here mapping MPC params to active sets
                Delta_prev = Delta(:, k:k+(ctrl.LaserMPC.N-1));
                
                if estim.use   % estimated states
                    if model.plant.nd == 0  %  no disturbnances option
                         [opt_out, feasible, info1, info2, info3, info4] =  ctrl.LaserMPC.optimizer{{xp, wa_prev, wb_prev, Price_prev, Delta_prev}}; % optimizer with estimated states
                    else
                         [opt_out, feasible, info1, info2, info3, info4] =  ctrl.LaserMPC.optimizer{{xp, Dpreview, wa_prev, wb_prev, Price_prev, Delta_prev}}; % optimizer with estimated states
                    end
                else    % perfect state update
                    if model.plant.nd == 0  %  no disturbnances option
                         [opt_out, feasible, info1, info2, info3, info4] =  ctrl.LaserMPC.optimizer{{x0, wa_prev, wb_prev, Price_prev}}; % optimizer with estimated states
                    else
                         [opt_out, feasible, info1, info2, info3, info4] =  ctrl.LaserMPC.optimizer{{x0, Dpreview, wa_prev, wb_prev, Price_prev}}; % optimizer with measured states  
                    end                 
                end

                MPC_options = ctrl.LaserMPC.optimizer.options;
                
            elseif ctrl.RMPC.use
    %             TODO: predictions as standalone function
                % preview of disturbance signals on the prediction horizon
                Dpreview = D(:, k:k+(ctrl.RMPC.Ndp-1));   
                % preview of thresholds on the prediction horizon - Dynamic comfort zone
                wa_prev = wa(:, k:k+(ctrl.RMPC.Nrp-1));
                wb_prev = wb(:, k:k+(ctrl.RMPC.Nrp-1));
                % preview of the price signal
                Price_prev = Price(:, k:k+(ctrl.RMPC.Nrp-1));
                if estim.use   % estimated states
                    if model.plant.nd == 0  %  no disturbnances option
                         [opt_out, feasible, info1, info2, info3, info4] =  ctrl.RMPC.optimizer{{xp, wa_prev, wb_prev, Price_prev}}; % optimizer with estimated states
                    else
                         [opt_out, feasible, info1, info2, info3, info4] =  ctrl.RMPC.optimizer{{xp, Dpreview, wa_prev, wb_prev, Price_prev}}; % optimizer with estimated states
                    end
                else    % perfect state update
                    if model.plant.nd == 0  %  no disturbnances option
                         [opt_out, feasible, info1, info2, info3, info4] =  ctrl.RMPC.optimizer{{x0, wa_prev, wb_prev, Price_prev}}; % optimizer with estimated states
                    else
                         [opt_out, feasible, info1, info2, info3, info4] =  ctrl.RMPC.optimizer{{x0, Dpreview, wa_prev, wb_prev, Price_prev}}; % optimizer with measured states  
                    end                 
                end
                  MPC_options = ctrl.RMPC.optimizer.options;
                  
                  
                  elseif ctrl.ARMPC.use
    %             TODO: predictions as standalone function
                % preview of disturbance signals on the prediction horizon
                Dpreview = D(:, k:k+(ctrl.ARMPC.Ndp-1));   
                % preview of thresholds on the prediction horizon - Dynamic comfort zone
                wa_prev = wa(:, k:k+(ctrl.ARMPC.Nrp-1));
                wb_prev = wb(:, k:k+(ctrl.ARMPC.Nrp-1));
                % preview of the price signal
                Price_prev = Price(:, k:k+(ctrl.ARMPC.Nrp-1));
                if estim.use   % estimated states
                    if model.plant.nd == 0  %  no disturbnances option
                         [opt_out_1, feasible_1, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer1{{xp_1, wa_prev, wb_prev, Price_prev}}; % optimizer with estimated states
                         [opt_out_2, feasible_2, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer2{{xp_2, wa_prev, wb_prev, Price_prev}}; % optimizer with estimated states
                         [opt_out_3, feasible_3, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer3{{xp_3, wa_prev, wb_prev, Price_prev}}; % optimizer with estimated states

                    else
                         [opt_out_1, feasible_1, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer1{{xp_1, Dpreview, wa_prev, wb_prev, Price_prev}}; % optimizer with estimated states
                         [opt_out_2, feasible_2, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer2{{xp_2, Dpreview, wa_prev, wb_prev, Price_prev}}; % optimizer with estimated states
                         [opt_out_3, feasible_3, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer3{{xp_3, Dpreview, wa_prev, wb_prev, Price_prev}}; % optimizer with estimated states

                    end
                else    % perfect state update
                    if model.plant.nd == 0  %  no disturbnances option
                         [opt_out_1, feasible_1, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer1{{x0, wa_prev, wb_prev, Price_prev}}; % optimizer with estimated states
                         [opt_out_2, feasible_2, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer2{{x0, wa_prev, wb_prev, Price_prev}}; % optimizer with estimated states
                         [opt_out_3, feasible_3, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer3{{x0, wa_prev, wb_prev, Price_prev}}; % optimizer with estimated states

                    else
                         [opt_out_1, feasible_1, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer1{{x0, Dpreview, wa_prev, wb_prev, Price_prev}}; % optimizer with measured states  
                         [opt_out_2, feasible_2, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer2{{x0, Dpreview, wa_prev, wb_prev, Price_prev}}; % optimizer with measured states  
                         [opt_out_3, feasible_3, info1, info2, info3, info4] =  ctrl.ARMPC.optimizer3{{x0, Dpreview, wa_prev, wb_prev, Price_prev}}; % optimizer with measured states  
  
                    end                 
                end
                %nv=3;% no of models
                
%                 if k==1
%                     for j=1:model.plant.ny
%                         if opt_out_1{1}(j)>=opt_out_2{1}(j)
%                             if opt_out_1{1}(j)>=opt_out_3{1}(j)
%                                 choix(j)=1;
%                             else
%                                 choix(j)=3;
%                             end
%                         elseif opt_out_2{1}(j)>=opt_out_3{1}(j)
%                             choix(j)=2;
%                         else
%                             choix(j)=3;
%                             
%                         end  
%                     end
%                     Q=1;
%                     choix=[1;1;1;1;1;1];
%                     choix=[2;2;2;2;2;2];
%                     choix=[3;3;3;3;3;3];
%                     outdata.choix(Q,:)=choix;
%                     opt_out=opt_out_3{1};%_2{1};
%                 end
                
%                 if (rem(k,15)==0)% to decide when the comparison should start
%                     for j=1:model.plant.ny
%                         if ee_1(j)>=0
%                             if ee_2(j)>=0
%                                 if ee_3(j)>=0
%                                     if ee_1(j)<=ee_2(j)
%                                         if ee_1(j)<=ee_3(j)
%                                             choix(j)=1;
%                                         else
%                                             choix(j)=3;
%                                         end
%                                     elseif ee_2(j)<=ee_3(j)
%                                         choix(j)=2;
%                                     else
%                                         choix(j)=3;
%                                     end
%                                 elseif ee_1(j)>ee_2(j)
%                                     choix(j)=2;
%                                 else
%                                     choix(j)=1;
%                                 end
%                             elseif ee_3(j)>=0
%                                 if ee_1(j)>ee_3(j)
%                                     choix(j)=3;
%                                 else
%                                     choix(j)=1;
%                                 end
%                             else
%                                 choix(j)=1;
%                             end
%                         elseif ee_2(j)>=0
%                             if ee_3(j)>=0
%                                 if ee_2(j)>ee_3(j)
%                                     choix(j)=3;
%                                 else
%                                     choix(j)=2;
%                                 end
%                             else
%                                 choix(j)=2;
%                             end
%                         elseif ee_3(j)>=0
%                             choix(j)=3;
%                         else
%                             if ee_1(j)<=ee_2(j)
%                                 if ee_1(j)<=ee_3(j)
%                                     choix(j)=1;
%                                 else
%                                     choix(j)=3;
%                                 end
%                             elseif ee_2(j)<=ee_3(j)
%                                 choix(j)=2;
%                             else
%                                 choix(j)=3;
%                             end
%                         end
%                     end
%                     Q=Q+1;
%                     outdata.choix(Q,:)=choix;
%                 end
                
                
%                % choix=[1;1;1;1;1;1];
%                 for jj=1:model.plant.ny
%                     eval(['opt_out(' num2str(jj) ')=opt_out_' num2str(choix(jj)) '{1}(' num2str(jj) ');' ])   
%                 end
                
                
%                 if (sum(choix==1))>=(sum(choix==2)) % to have uniform states, not different dynamics for each one.
%                     if (sum(choix==1))>=(sum(choix==3))
%                         opt_out=opt_out_1{1};%
%                     else
%                         opt_out=opt_out_3{1};
%                     end
%                 elseif (sum(choix==2))>=(sum(choix==3))
%                     opt_out=opt_out_2{1};
%                 else
%                     opt_out=opt_out_3{1};
%                 end
%                 opt_out=opt_out_3{1};
                % IC
                % duration to collect correct data to make a comparison 
               
                if k<(N*4)
                    if (rem(k,N)==0)
                        choix=choix-1;
                        if choix==0
                            choix=3;
                        end
                    end
                   
                end
                % choose the minimum error depending on the data 
                if k==N*4
                    if sum(sum(abs(outdata.ep_1(44:65,:)))) < sum(sum(abs(outdata.ep_2(22:43,:))))
                       if sum(sum(abs(outdata.ep_1(44:65,:))))< sum(sum(abs(outdata.ep_3(66:87,:))))
                           choix = 1;
                       else
                           choix = 3;
                       end
                    elseif sum(sum(abs(outdata.ep_2(22:43,:))))< sum(sum(abs(outdata.ep_3(66:87,:))))
                        choix=2;
                    else
                        choix=3;
                    end
                end
                
                if choix==3
                    opt_out=opt_out_3{1};
                elseif choix==2
                    opt_out=opt_out_2{1};
                elseif choix==1
                    opt_out=opt_out_1{1};
                end
                
                 %opt_out=opt_out_2{1};
                
              
                
%                 if k==22
%                     opt_out=opt_out_2{1};
%                     %feasible=feasible_1;
%                     %MPC_options = ctrl.ARMPC.optimizer1.options;
%                 elseif select==44
%                     opt_out=opt_out_1{1};
%                     %feasible=feasible_2;
%                     %MPC_options = ctrl.ARMPC.optimizer2.options;
%                 elseif select==66
%                     opt_out=opt_out_3{1};
%                     %feasible=feasible_3;
%                     %MPC_options = ctrl.ARMPC.optimizer3.options;
%                      elseif select==66
%                 else
%                 end
                    
                
                  MPC_options = ctrl.ARMPC.optimizer1.options;
                  
                  
                  
           elseif ctrl.RMPCLMI.use
    %     TODO: predictions as standalone function
                % preview of disturbance signals on the prediction horizon
                Dpreview = D(:, k:k+(ctrl.RMPCLMI.Ndp-1));   
                % preview of thresholds on the prediction horizon - Dynamic comfort zone
                wa_prev = wa(:, k:k+(ctrl.RMPCLMI.Nrp-1));
                wb_prev = wb(:, k:k+(ctrl.RMPCLMI.Nrp-1));
                % preview of the price signal
                Price_prev = Price(:, k:k+(ctrl.RMPCLMI.Nrp-1));
                
                %Steady state conditions:
                re = 293 * ones(model.pred.ny,1);
                Xss = M1 * [-(model.pred.Ed*d0 + model.pred.Gd); [R(:,k) - model.pred.Fd]];
                Uss = M2 * [-(model.pred.Ed*d0 + model.pred.Gd); [R(:,k) - model.pred.Fd]];
                
                if estim.use  %  no disturbnances option
                    Xer = xp - Xss;% X(:,1): zeros
                else
                    Xer = x0 - Xss;% X(:,1): zeros
                end
                 
                u = zeros(model.pred.nu,1);
                Ue = u - Uss;%
                
                ymaxred = wb(:,k);% L4 maybe use wb
                %ymaxred = 300*ones(model.pred.ny,1);
                
                umax = model.pred.umax - Uss;
                ymax = ymaxred - (model.pred.Cd*Xss + model.pred.Dd*Uss + model.pred.Fd);
                
 
                %     initialize LMI MPC diagnostics vectors
                [solutions, feasible, info1, info2, info3, info4] =  ctrl.RMPCLMI.optimizer{{Xer,umax,ymax}}; % optimizer with estimated states
                
                Ynew = solutions{1};
                Qnew = solutions{2};
                gamma = solutions{3};
                F = Ynew*inv(Qnew);
                
                u = Uss + F*Xer;
                opt_out{1} = u;                
                MPC_options = ctrl.RMPCLMI.optimizer.options;
                
            end
                
           if SimParam.breakdown ==1
                if SimParam.run.start>=180
                    if(k>=320)&&(k<=320+24) %24 = 6 hours as K = 15mins
                        opt_out= 0.25 * model2.pred.umax;% summer
                    end
                else
                    if (k>=480)&&(k<=480+24)% winter
                        opt_out=zeros(6,1);% winter
                    end
                end
            end
%             %     feasibility check
           %     feasibility check
            if ~ismember(feasible_2, [0 3 4 5])
                k
               error('infeasible')      
            else
                if ctrl.ARMPC.use
                    uopt=opt_out;%_2{1};
                else
                uopt = opt_out{1};   % optimal control action
                end
%                OBJ(k) =  opt_out{2};   % objective function value   
%                 DUALS(:,k) = info2{1};    % dual variables
%                DUALS(:,k) = info4.Dual; % dual variables values
 %               PRIMALS(:,k) = info4.Primal; % primal variables values             
  %              SolverTime(k) = info4.solvertime;  %  elapsed solver time of one MPC step               
                if strcmp(MPC_options.solver,'+quadprog')
                    ITERS(:,k) = info4.solveroutput.output.iterations;
                    INEQLIN(:,k) = info4.solveroutput.lambda.ineqlin;
                    EQLIN(:,k) = info4.solveroutput.lambda.eqlin;
                end
%                 info4.solveroutput.lambda              
            end 
            
            
        elseif ctrl.MLagent.use   % machine learning controller
%             TODO: finish implementation of all ML ctrls
            if ctrl.MLagent.TDNN.use    % Time delay neural network           
                InputDelays = ctrl.MLagent.TDNN.net_apply.numInputDelays; % N-1
                
                if k > InputDelays
% features for TDNN             
%                     Dpreview = D(ctrl.MLagent.use_D, k:k+InputDelays);   
%                     wa_prev = wa(1, k:k+InputDelays);
%                     wb_prev = wb(1, k:k+InputDelays);
%                     y_past = Y(:,k-InputDelays:k-1);
%                      uopt =  TDNN_ctrl([yn; Dpreview(:,end); wb_prev(:,end)],...                              
%                      [ y_past; Dpreview(:,1:end-1); wb_prev(:,1:end-1)]);
%                      

%   current code - Working
                     uopt =  TDNN_ctrl([yn; D(ctrl.MLagent.use_D, k+InputDelays);wb(1,k+InputDelays)],...                              
                     [ Y(:,k-InputDelays:k-1); D(ctrl.MLagent.use_D, k:k+(InputDelays-1));wb(1,k:k+(InputDelays-1))]);

% previous code - Working
%                      uopt =  NN_TS_ctrl([yn; D(ctrl.MLagent.use_D, k+InputDelays);wb(1,k+InputDelays)],...                              
%                      [ Y(:,k-InputDelays:k-1); D(ctrl.MLagent.use_D, k:k+(InputDelays-1));wb(1,k:k+(InputDelays-1))]);

                    uopt(abs(uopt)<200) = 0;
%                     control contraints
                    uopt(uopt>model_1.pred.umax) = model_1.pred.umax(uopt>model_1.pred.umax);
                    uopt(uopt<model_1.pred.umin) = model_1.pred.umin(uopt<model_1.pred.umin);
                    
%                     block simultaneous heating and cooling
                    if not(all(uopt < 0) || all(uopt > 0))                       
                       if sum(abs(uopt)) < 1e3
                          uopt = zeros(model_1.pred.nu,1);
                       elseif sum(uopt) > 0
                          uopt(uopt<0) = 0;
                       else
                          uopt(uopt>0) = 0;
                       end
                    end                   
                else
                    uopt = zeros(model_1.pred.nu,1);
                end
                
            elseif ctrl.MLagent.RT.use
%     TODO:
                
            end     
        end
    end  
   
    
%% Controlled Sytem Dynamics
% TODO: wrap simulation and measurement in standalone function with standardized interface
    
% 1, Pre-computed controls
    if not(ctrl.use)
       uopt = U(:,k);       % current controls 
    end
         
% 2, EMULATOR - plant model
%     TODO: dymola co-simulation
    if  SimParam.emulate
%    State and Output update
        WW(41,1)=W(k);
       
        xn = model.plant.Ad*x0 + model.plant.Bd*uopt+ model.plant.Ed*d0 + model.plant.Gd*1 ;%+ model.plant.Ed* WW;
        yn = model.plant.Cd*x0 + model.plant.Dd*uopt + model.plant.Fd*1;

        % simulation model data vectors
        X(:,k+1) = xn;
        Y(:,k) = yn;
        U(:,k) = uopt;       
    else   
% 3, Output measurements - real plant
    % TODO: implement real time measurement
       yn = Y(:,k);         % current outputs               
    end


%%   ESTIMATION  
% TODO standalone functions?
    if estim.use 
        if estim.SKF.use  % stationary KF
            
            % measurement update                              
            yp = model_1.pred.Cd*xp + model_1.pred.Dd*uopt + model_1.pred.Fd*1;          % output estimation
            ep = yn - yp;                                                       % estimation error
            xe = xp  + estim.SKF.L1*ep;                                       % estimated state
            
            % time update
            xp = model_1.pred.Ad*xe + model_1.pred.Bd*uopt + model_1.pred.Ed*d0 + model_1.pred.Gd*1;
            ye = model_1.pred.Cd*xe + model_1.pred.Dd*uopt + model_1.pred.Fd*1;     % output estimate with x[n|n]
             
        elseif estim.TVKF.use  % time varying KF           
           if ctrl.ARMPC.use 
               uopt_1 = opt_out_1{1};
               uopt_2 = opt_out_2{1};
               uopt_3 = opt_out_3{1};
               %% Estimator_1
               if k == 1
                  P_1 = model_1.pred.Bd*estim.TVKF.Qe*model_1.pred.Bd';% Estimator_1        % Initial error covariance   
                  P_2 = model_2.pred.Bd*estim.TVKF.Qe*model_2.pred.Bd';% Estimator_2        % Initial error covariance 
                  P_3 = model_3.pred.Bd*estim.TVKF.Qe*model_3.pred.Bd';% Estimator_3        % Initial error covariance 
               end
            % Measurement update   
            L_1 = P_1*model_1.pred.Cd'/(model_1.pred.Cd*P_1*model_1.pred.Cd'+estim.TVKF.Re); % observer gain
            L_2 = P_2*model_2.pred.Cd'/(model_2.pred.Cd*P_2*model_2.pred.Cd'+estim.TVKF.Re); % observer gain
            L_3 = P_3*model_3.pred.Cd'/(model_3.pred.Cd*P_3*model_3.pred.Cd'+estim.TVKF.Re); % observer gain
            
            yp_1 = model_1.pred.Cd*xp_1 + model_1.pred.Dd*uopt_1 + model_1.pred.Fd*1;% output estimation
            yp_2 = model_2.pred.Cd*xp_2 + model_2.pred.Dd*uopt_2 + model_2.pred.Fd*1;% output estimation
            yp_3 = model_3.pred.Cd*xp_3 + model_3.pred.Dd*uopt_3 + model_3.pred.Fd*1;% output estimation

              ep_1 = yn-yp_1;%yn - yp_1;                                                       % estimation error
              ep_2 = yn-yp_2;%yn - yp_2;                                                       % estimation error
              ep_3 = yn-yp_3;%yn - yp_3; 295.9                                                      % estimation error
              
              %outdata.ep_1=ep_1;
              %outdata.ep_2=ep_2;
              %outdata.ep_3=ep_3;
              
              xe_1 = xp_1 + L_1*ep_1;                                                    % x[n|n]
              xe_2 = xp_2 + L_2*ep_2;                                                    % x[n|n]
              xe_3 = xp_3 + L_3*ep_3;                                                    % x[n|n]
              
              P_1 = (eye(model_1.pred.nx)-L_1*model_1.pred.Cd)*P_1;                          % P[n|n]   estimation error covariance
              P_2 = (eye(model_2.pred.nx)-L_2*model_2.pred.Cd)*P_2;                          % P[n|n]   estimation error covariance
              P_3 = (eye(model_3.pred.nx)-L_3*model_3.pred.Cd)*P_3;                          % P[n|n]   estimation error covariance
             
              errcov_1 = model_1.pred.Cd*P_1*model_1.pred.Cd';                              % output estimation error covariance
              errcov_2 = model_2.pred.Cd*P_2*model_2.pred.Cd';                              % output estimation error covariance
              errcov_3 = model_3.pred.Cd*P_3*model_3.pred.Cd';                              % output estimation error covariance
              
              % Time update
              xp_1 = model_1.pred.Ad*xe_1 + model_1.pred.Bd*uopt_1 + model_1.pred.Ed*d0 + model_1.pred.Gd*1;        % x[n+1|n]
              xp_2 = model_2.pred.Ad*xe_2 + model_2.pred.Bd*uopt_2 + model_2.pred.Ed*d0 + model_2.pred.Gd*1;        % x[n+1|n]
              xp_3 = model_3.pred.Ad*xe_3 + model_3.pred.Bd*uopt_3 + model_3.pred.Ed*d0 + model_3.pred.Gd*1;        % x[n+1|n]
              
              P_1 = model_1.pred.Ad*P_1*model_1.pred.Ad' + model_1.pred.Bd*estim.TVKF.Qe*model_1.pred.Bd';       % P[n+1|n]
              P_2 = model_2.pred.Ad*P_2*model_2.pred.Ad' + model_2.pred.Bd*estim.TVKF.Qe*model_2.pred.Bd';       % P[n+1|n]
              P_3 = model_3.pred.Ad*P_3*model_3.pred.Ad' + model_3.pred.Bd*estim.TVKF.Qe*model_3.pred.Bd';       % P[n+1|n]

            
              ye_1 = model_1.pred.Cd*xe_1 + model_1.pred.Dd*uopt_1 + model_1.pred.Fd*1;     % output estimate with x[n|n]
              ye_2 = model_2.pred.Cd*xe_2 + model_2.pred.Dd*uopt_2 + model_2.pred.Fd*1;     % output estimate with x[n|n]
              ye_3 = model_3.pred.Cd*xe_3 + model_3.pred.Dd*uopt_3 + model_3.pred.Fd*1;     % output estimate with x[n|n]

              ee_1 = yn - yp_1;                                                       % estimation error
              ee_2 = yn - yp_2;                                                       % estimation error
              ee_3 = yn - yp_3;                                                       % estimation error
             
             if choix==3
                  outdata.ep_3(k,:)=ep_3;
             elseif choix ==2
                  outdata.ep_2(k,:)=ep_2;
              elseif choix==1
                  outdata.ep_1(k,:)=ep_1;
              end
               
              
              
%               ee_1 = R(:,k) - ye_1;                                                       % estimation error
%               ee_2 = R(:,k) - ye_2;                                                       % estimation error
%               ee_3 = R(:,k) - ye_3;  
%               
%               ee_1 = -yp_1 + (wa_prev(:,1)+wb_prev(:,1))/2;                                                       % estimation error
%               ee_2 = -yp_2 + (wa_prev(:,1)+wb_prev(:,1))/2;                                                       % estimation error
%               ee_3 = -yp_3 + (wa_prev(:,1)+wb_prev(:,1))/2;
%               
              % time varying parameters data
              %EstimGain{k} = L1;
              %ErrorCovar{k} = errcov;
              outdata.YP1(k,:) = yp_1;
              outdata.YP2(k,:) = yp_2;
              outdata.YP3(k,:) = yp_3;
              
              
           
           else    
              if k == 1
                  P = model_1.pred.Bd*estim.TVKF.Qe*model_1.pred.Bd';         % Initial error covariance   
              end
            
              % Measurement update
              L1 = P*model_1.pred.Cd'/(model_1.pred.Cd*P*model_1.pred.Cd'+estim.TVKF.Re); % observer gain
              yp = model_1.pred.Cd*xp + model_1.pred.Dd*uopt + model_1.pred.Fd*1;          % output estimation
              ep = yn - yp;                                                       % estimation error
              xe = xp + L1*ep;                                                    % x[n|n]
              P = (eye(model_1.pred.nx)-L1*model_1.pred.Cd)*P;                          % P[n|n]   estimation error covariance
              errcov = model_1.pred.Cd*P*model_1.pred.Cd';                              % output estimation error covariance
              
              % Time update
              xp = model_1.pred.Ad*xe + model_1.pred.Bd*uopt + model_1.pred.Ed*d0 + model_1.pred.Gd*1;        % x[n+1|n]
              P = model_1.pred.Ad*P*model_1.pred.Ad' + model_1.pred.Bd*estim.TVKF.Qe*model_1.pred.Bd';       % P[n+1|n]
            
              ye = model_1.pred.Cd*xe + model_1.pred.Dd*uopt + model_1.pred.Fd*1;     % output estimate with x[n|n]
              
              % time varying parameters data
              EstimGain{k} = L1;
              ErrorCovar{k} = errcov;
           end
           
        elseif estim.MHE.use  % TODO: moving horizon estimation
   
            if k >= estim.MHE.N
             
                N = estim.MHE.N;
                                
                [opt_out, feasible, info1, info2] = estim.MHE.optimizer{{Y(:,k-N+1:k), U(:,k-N+1:k), D(:,k-N+1:k), Xp(:,k-N+1)}}; % optimizer with estimated states
               
                xe = opt_out{1};    % estimated state at x[n-N+1|n]
                ve =  opt_out{2};   % v decision variables at  x[n-N+1:n|n]
                obj_estim =  opt_out{3}; 
                
                if estim.MHE.Condensing                                       
                    we = zeros(model_1.pred.nx,estim.MHE.N);
                else
                    we =  opt_out{4};   % w decision variables at  x[n-N+1:n|n]
                end                          
                
            %    MHE post processing, integration of states at x[n-N+1|n] via state
            %    update and w to get x[n|n]           
                for j = 1:N
                    xe = model_1.pred.Ad*xe + model_1.pred.Bd*U(:,k-N+j) + model_1.pred.Ed*D(:,k-N+j) + model_1.pred.Gd*1 + we(:,j);
                end
             
                ye = model_1.pred.Cd*xe + model_1.pred.Dd*uopt + model_1.pred.Fd*1 + ve(:,N);     % output estimate with x[n|n]               
                
                OBJ_MHE(:,k) = obj_estim;
                We(:,k) = we(:,N);
                Ve(:,k) = ve(:,N);
            else
%              TODO:   put KF or growing horizon implementation for initial
%              estimate
                                                      
            end     
        end

    %     estimator data
        if ctrl.ARMPC.use 
        Xp_1(:,k+1) = xp_1;
        Xp_2(:,k+1) = xp_2;
        Xp_3(:,k+1) = xp_3;
        else
        Xp(:,k+1) = xp;  
        end
        Xe(:,k) = xe;
        Ye(:,k) = ye;
        Yp(:,k) = yp;
    end
    
    
%% Control action post-processing
% TODO - heat flows to valve positions
% Q = m*cp*p*(T_sup - T_return)
  
    %% ---------- Comfort evaluation ----------
    if  ctrl.use
        % VIOLATIONS of the thermal comfort zones of individual outputs
        va = zeros(model.plant.ny,1); vb = zeros(model.plant.ny,1); v = zeros(model.plant.ny,1);
        % VIOLATIONS of the PMV zone of individual outputs
        PMVva = zeros(model.plant.ny,1); PMVvb = zeros(model.plant.ny,1); PMVv = zeros(model.plant.ny,1);

        %     violation evaluation for j-th output
        for j = 1:model.plant.ny
            if yn(j) > wa(j,k) + SimParam.comfortTol    % above viol. condition
                v(j) = yn(j) - wa(j,k);        % above viol. magnitude
                va(j) = v(j);
            elseif yn(j)  <  wb(j,k) - SimParam.comfortTol  % below viol. condition
                v(j) = yn(j) - wb(j,k);           % below viol. magnitude
                vb(j) = v(j);
            end

            % PMV index for each zone, with Tr = 29 deg C
            PMV(j,k) = pmv_iso(yn(j)-273.15, 29);
            if  PMV(j,k) > PMVub(k)    % above PMV viol. condition
                PMVv(j) = PMV(j,k) - PMVub(k);        % above viol. magnitude
                PMVva(j) = PMVv(j);
            elseif  PMV(j,k)  <  PMVlb(k)  % below PMV viol. condition
                PMVv(j) = PMV(j,k) - PMVlb(k);        % above viol. magnitude
                PMVvb(j) = PMVv(j);
            end
        end
        %  comfort zone violations vectors
        Viol(:,k) = v;
        AboveViol(:,k) = va;
        BelowViol(:,k) = vb;
        % PMV violations vectors
        PMVViol(:,k) =  PMVv;
        PMVAboveViol(:,k) =  PMVva;
        PMVBelowViol(:,k) =  PMVvb;
    end
           
%     REMAINING simulation time computation
    step_time = etime(clock, start_t);                  %  elapsed time of one sim. step
    StepTime(k) = step_time;
    av_step_time = step_time/k;                         % average_step_time
    rem_sim_time = av_step_time*(Nsim-k);           % remaining_sim_time
     
    msg = sprintf('\n*** estimated remaining simulation time = %.2f sec \n',rem_sim_time);    % statement   
    fprintf([reverseStr, msg]);                                                 % print statement
    reverseStr = repmat(sprintf('\b'), 1, length(msg));                         % clear line   
        
end

%% ---------- Simulation Output Data  ------------
Ts = model.plant.Ts;

if  ctrl.use
    % -------- ENERGY COSTS ----------
    Uheat = U(U>0);
    Ucool = U(U<0);
    Qheat = 1;   % heat coefficient
    Qcool = 1;   % cool coefficient
    %  ENERGY COSTS for individudal inputs
        for j = 1:model.plant.ny
            outdata.info.HeatingCost(j) = sum(Qheat*Uheat(j:model.plant.ny:size(Uheat,1)))*Ts/1000/3600;       % heating cost [kW hours]
            outdata.info.CoolingCost(j) = sum(abs(Qcool*Ucool(j:model.plant.ny:size(Ucool,1))))*Ts/1000/3600;  % cooling cost [kW hours]
            outdata.info.TotalCost(j) = outdata.info.HeatingCost(j)+outdata.info.CoolingCost(j);             % total cost [kW hours]
        end
    % Overall ENERGY COST
    outdata.info.OverallHeatingCost = sum(outdata.info.HeatingCost);                           % heating cost [kW hours]
    outdata.info.OverallCoolingCost = sum(outdata.info.CoolingCost);                           % cooling cost [kW hours]
    outdata.info.OverallTotalCost = sum(outdata.info.OverallHeatingCost)+sum(outdata.info.OverallCoolingCost);     % total cost [kW hours]
end

% ------------ COMFORT -----------------
% COMFORT satisfaction for each output
if  ctrl.use
    for j = 1:model.plant.ny
        %  comfort satisfacion
        outdata.info.PositiveComfortRate(j) = 100*(1-nnz(AboveViol(j,:))/length(AboveViol(j,:)));   %  PCR
        outdata.info.NegativeComfortRate(j) = 100*(1-nnz(BelowViol(j,:))/length(BelowViol(j,:)));   %  NCR
        outdata.info.TotalComfortRate(j) = 100*(1-nnz(Viol(j,:))/length(Viol(j,:)));                %  TCR
        %   sum of comfort zone violations
        outdata.info.PositiveViolSum(j) =  norm(AboveViol(j,:), 1);  % sum of positive viol - PVS
        outdata.info.NegativeViolSum(j) =  norm(BelowViol(j,:), 1);  % sum of negative viol - NVS
        outdata.info.TotalViolSum(j) =  norm(Viol(j,:), 1);          % sum of total viol - TVS
        %   maximum of comfort zone violation
        outdata.info.PositiveViolMax(j) =  norm(AboveViol(j,:), Inf); % max of positive viol - PVM
        outdata.info.NegativeViolMax(j) =  norm(BelowViol(j,:), Inf); % max of negative viol - NVM
        outdata.info.TotalViolMax(j) =  norm(Viol(j,:), Inf);         % max of total viol - TVM
        %  kelvin hours
        outdata.info.KelvinHours(j) =  norm(Viol(j,:), 1)*900/3600;          % Kh
         %   sum of PMV violations
        outdata.info.PMVPositiveViolSum(j) =  norm(PMVAboveViol(j,:), 1);  % sum of positive viol - PVS
        outdata.info.PMVNegativeViolSum(j) =  norm(PMVBelowViol(j,:), 1);  % sum of negative viol - NVS
        outdata.info.PMVTotalViolSum(j) =  norm(PMVViol(j,:), 1);          % sum of total viol - TVS
        %   maximum of PMV violation
        outdata.info.PMVPositiveViolMax(j) =  norm(PMVAboveViol(j,:), Inf); % max of positive viol - PVM
        outdata.info.PMVNegativeViolMax(j) =  norm(PMVBelowViol(j,:), Inf); % max of negative viol - NVM
        outdata.info.PMVTotalViolMax(j) =  norm(PMVViol(j,:), Inf);         % max of total viol - TVM
    end
% Overall COMFORT zone satisfaction
outdata.info.Overall_PCR = sum(outdata.info.PositiveComfortRate)/length(outdata.info.PositiveComfortRate);  % positive
outdata.info.Overall_NCR = sum(outdata.info.NegativeComfortRate)/length(outdata.info.NegativeComfortRate);  % negative
outdata.info.Overall_TCR = sum(outdata.info.TotalComfortRate)/length(outdata.info.TotalComfortRate);        % total
% Overall sum of comfort zone violations
outdata.info.Overall_PVS = sum(outdata.info.PositiveViolSum);  % sum of positive viol
outdata.info.Overall_NVS = sum(outdata.info.NegativeViolSum);  % sum of negative viol
outdata.info.Overall_TVS = sum(outdata.info.TotalViolSum);     % sum of total viol
% Overall maximum of comfort zone violations
outdata.info.Overall_PVM =  max(outdata.info.PositiveViolMax);  % max of positive viol
outdata.info.Overall_NVM =  max(outdata.info.NegativeViolMax);  % max of negative viol
outdata.info.Overall_TVM =  max(outdata.info.TotalViolMax);     % max of total viol
% K/hour metric per zone
outdata.info.Overall_PV_metric = outdata.info.Overall_PVS/(Nsim*(Ts/3600))/model.plant.ny;  % K/h metric of positive viol
outdata.info.Overall_NV_metric = outdata.info.Overall_NVS/(Nsim*(Ts/3600))/model.plant.ny;  % K/h metric of negative viol
outdata.info.Overall_TV_metric = outdata.info.Overall_TVS/(Nsim*(Ts/3600))/model.plant.ny;  % K/h metric of total viol
% Overall kelvin hours
outdata.info.Overall_Kh = sum(outdata.info.KelvinHours);     % sum of kelvin hours
% Overall sum of PMV violations
outdata.info.Overall_PMV_PVS = sum(outdata.info.PMVPositiveViolSum);  % sum of positive viol
outdata.info.Overall_PMV_NVS = sum(outdata.info.PMVNegativeViolSum);  % sum of negative viol
outdata.info.Overall_PMV_TVS = sum(outdata.info.PMVTotalViolSum);     % sum of total viol
% Overall maximum of PMV violations
outdata.info.Overall_PMV_PVM =  max(outdata.info.PMVPositiveViolMax);  % max of positive viol
outdata.info.Overall_PMV_NVM =  max(outdata.info.PMVNegativeViolMax);  % max of negative viol
outdata.info.Overall_PMV_TVM =  max(outdata.info.PMVTotalViolMax);     % max of total viol
end

% ------------ DATA -----------------

% SIMILATION STRUCTURES
outdata.model = model;    %  model 
outdata.estim = estim;    % estimator
outdata.ctrl = ctrl;      %  controller
% outdata.dist = dist;      %  disturbances
% outdata.dist = refs;      %  references
outdata.SimParam = SimParam;        %  simulation parameters
outdata.SimParam.run.Nsim = Nsim;    % sim steps

% plant simulation data 
outdata.data.X = X;         %  state vector
outdata.data.Y = Y;         %  output vector
outdata.data.U = U;         %  input vector
outdata.data.D = D;         %  disturbance vector

% estimator data
if estim.use ==1
    outdata.data.Xe = Xe;         %  estimated state vector  [n|n]
    outdata.data.Ye = Ye;         %  estimated output vector [n|n]
    outdata.data.Xp = Xp_1;         %  previous estimaror state vector [n|n-1]%%%%% note L2020
    outdata.data.Yp = Yp;         %  estimated output vector [n|n-1]
    
    if estim.TVKF.use
        outdata.data.EstimGain = EstimGain;
        outdata.data.ErrorCovar = ErrorCovar;
        
    elseif estim.MHE.use
        outdata.data.We = We;         
        outdata.data.Ve = Ve;    
    end

end

% % TODO: integrate this
if ctrl.use
    % referecne + comfort zone
    % outdata.data.R = refs_mpc;  %  references vector
    outdata.data.wa = wa;       %  above threshold
    outdata.data.wb = wb;       %  below threshold
%     % PMV zone
%     outdata.data.PMVub = PMVub; %  above threshold
%     outdata.data.PMVlb = PMVlb; %  below threshold
%     %  comfot zone violations
%     outdata.data.AV = AboveViol;
%     outdata.data.BV = BelowViol;
%     outdata.data.V = Viol;
%     % PMV index profiles
%     outdata.data.PMV = PMV;
%     %  PMV zone violations
%     outdata.data.PMV_AV = PMVAboveViol;
%     outdata.data.PMV_BV = PMVBelowViol;
%     outdata.data.PMV_V = PMVViol;
    
%     Price signal
    outdata.data.Price = Price(:,1:end-Nrp);       
    outdata.data.Cost = Price(:,1:end-Nrp).*U;   
    
    if ctrl.MPC.use || ctrl.LaserMPC.use   
        outdata.solver.OBJ = OBJ; % obj function
        outdata.solver.DUALS = DUALS; % dual variables
        outdata.solver.PRIMALS = PRIMALS;  % primal variables
%        outdata.solver.SolverTime = SolverTime;  % solvertime
        outdata.solver.MPC_options.solver = MPC_options.solver; % solver info
        if strcmp(MPC_options.solver,'+quadprog')
            outdata.solver.ITERS = ITERS;  % number of iterations
            outdata.solver.INEQLIN = INEQLIN;  % inequalities
            outdata.solver.EQLIN = EQLIN;     % equalities  
        end
    end
end

% elapsed time of simulation
outdata.info.SimTime = etime(clock, start_t);
outdata.info.StepTime = StepTime;
    

%% Simulation results reports and plots

% print verbose condition
if SimParam.verbose
        % -------- PRINT --------------
        fprintf('\n------------------ Simulation Results ---------------\n');
        
    if ctrl.use
        %  energy cost
        fprintf('          Heating cost: %.2f kWh\n', outdata.info.OverallHeatingCost);
        fprintf('          Cooling cost: %.2f kWh\n', outdata.info.OverallCoolingCost);
        fprintf('            Total cost: %.2f kWh\n', outdata.info.OverallTotalCost);
        fprintf('               Comfort: %.2f Kh\n',  outdata.info.Overall_Kh);
        fprintf('        PMV violations: %.2f \n', outdata.info.Overall_PMV_TVS);
    end
    
        % compuation time
        fprintf('*** Simulation Time: %.1f secs\n', outdata.info.SimTime);
    
end
% Simulation ends  
outdata.info.date =  datestr(now);
fprintf('*** Simulation finished at: %s \n',  outdata.info.date);

% SAVE data 
if SimParam.flagSave
    str = sprintf('../Data/outData%s_from%d_to%d.mat', model.buildingType, SimParam.run.start, SimParam.run.end);
    save(str,'outdata');
end

% PROFILE CPU load
if SimParam.profile
    outdata.profile = profile('info');
    profile viewer
end


end