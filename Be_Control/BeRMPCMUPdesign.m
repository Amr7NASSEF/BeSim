% ---------------- %
%
% Be_RMPC_MUP
%
% ---------------- %

% 
%
% ------------------------------------------------------------------------------

function [mpc, constraints_info] = BeRMPCMUPdesign(model, MPCParam)

%% trying to adapt this way to be used inside BeSIM 

%first we need nx so
%% MPC parameters

    % dimensions
    nx = model.uncert.nx;
    ny = model.uncert.ny;                             % nd = model.pred.nd; % maybe not needed 
    nu = model.uncert.nu;
    nv = model.uncert.nv; 
    A=model.uncert.Ad;
    B=model.uncert.Bd;
    C=model.uncert.Cd;
    Wx=MPCParam.Qx;
    Wu=MPCParam.Qu;
    u_max=model.uncert.umax;
    x_max=303*ones(1,model.uncert.ny); % output constraints should come from wa (:;1:Nrp) coming from ref.wb( start- stop simultaion)
    Ts=model.uncert.Ts;
    %% initial conditions 
    
    % x0 = [3;0];
    
    %% Soft-Constraints Params
%
u_sl = 0.4;% check
y_sl = [1;0.5];%check
Wsu = Wu*1e4;
Wsy = Wx*1e2;
Esu = eye(nu);
Esy = eye(ny);
%
param.u_sl = u_sl;
param.y_sl = y_sl;
param.Wsu = Wsu;
param.Wsy = Wsy;
param.Esu = Esu;
param.Esy = Esy;

%% on/off Feasibility Chcek
chk_feas = 'off'; % Enable Feasibility Chcek
% chk_feas = 'off'; % Disable Feasibility Chcek
    
  %% Select RMPC method Using CLI
[rmpc_method,rmpc_kwd] = mup_cli_rmpc_method;

%% Additional RMPC Tunning Parameter
if( (isequal(rmpc_method,'Huang et al. (2011)')) | (isequal(rmpc_method,'Wan et Huang ((2015))')) | (isequal(rmpc_method,'Mao et Huang (2015)')) )
    beta=1e3;
end % if


%% Step 3: SDP Formulation
mup_sdp


%% Step 4: RMPC_OPTIMIZER Design

% Construct the RMPC_OPTMIZER and Store Data into structures DESIGN, MODEL, PROBLEM, SETUP, RMPC_BLOCK_WS[GLOBAL]
mpc = optimizer(constr_optimizer,obj,sdpsettings('verbose',0),xk, out(:));
%constraints_info=constr_optimizer

constraints_info.con = constr_optimizer;
constraints_info.size = size(constr_optimizer);
constraints_info.i_length = NaN(length(constr_optimizer),1);
constraints_info.equality = is(constr_optimizer,'equality'); 

