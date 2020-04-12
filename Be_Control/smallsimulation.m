clear all;
yalmip('clear');
ModelParam.Orders.range = [4, 7, 10, 15, 20, 30, 40, 100];
ModelParam.Orders.choice = 7;%'full'; % L2020 March 4 'full';                            % model order selection for prediction
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
model_Light = BeModel('RenoLight', ModelParam);      %3 construct a model object   
model_Reno = BeModel('Reno', ModelParam);      %2 construct a model object   
model_Old = BeModel('Old', ModelParam);      %1 construct a model object   

model = model_Old;      % construct a model for prediction   

%% Disturbacnes 
% ambient temperature, solar radiation, internal heat gains
DistParam.reload = 0;
dist = BeDist(model, DistParam);
mpc=BeRMPCLMI(model_Old,model_Reno,model_Light);


%% ----------------- 

% 
% 
%  %% MPC parameters
%     % dimensions
%     nx = model1.pred.nx;
%     ny = model1.pred.ny;
%     nd = model1.pred.nd;
%     nu = model1.pred.nu;
% 
%     % variables
%     x = sdpvar(nx, 1, 'full'); % states of the building
%     W = sdpvar(nx,nx, 'symmetric'); % F = Y * W^-1  where W > 0 and Y are obtained from the solution (if it exists) to the following linear objective minimization min Gamma
%     Y = sdpvar(nu,nx, 'full'); % from the above equation.
%     umax = sdpvar(nu,1);
%     ymax = sdpvar(ny,1);
%     Gamma = sdpvar(1,1);% minimize Gamma or (1,1)
%     s = sdpvar(ny,1); % maximum output constraint slack.
%       
% 
%     
%     %% Models
%     %  objective function+ constraints init
%     obj = 0;
%     con = [];
%     %
%     nv=3; % number of vertices 
%     A{1,1} =  model1.pred.Ad;
%     B{1,1} =  model1.pred.Bd;
%     C{1,1} =  model1.pred.Cd;
%     A{2,1} =  model2.pred.Ad;
%     B{2,1} =  model2.pred.Bd;
%     C{2,1} =  model2.pred.Cd;
%     A{3,1} =  model3.pred.Ad;
%     B{3,1} =  model3.pred.Bd;
%     C{3,1} =  model3.pred.Cd;
%     
%    %% Weight Matrices
%    
%     Qy = eye(nu)*10^-3;
%     Qw = C{1,1}'*C{1,1}*10^5;
%     %Qw = Qx'*Qx*10^5;
%  
%     % to organise the matrices for LMI 
%     ZEROx = zeros(nx,nx);
%     ZEROux = zeros(nu,nx);
%     ZEROxu = zeros(nx,nu);
%     Ix = eye(nx);
%     Iu = eye(nu);
% 
%         %   -------------  Constraints  -------------
%     Lmi_Lyap=[[W >= 0] : ['Constraint on W']];
%     Lmi_rie =[[[1, x'; x, W] >= 0] :['constraint on X'] ];
% 
% % LMI for convix
%     Lmi_convix = [];
%     for v = 1 : nv
%       lmi_conv_item = [[ 
%          [ W  ,  (A{v}*W + B{v}*Y)' , (sqrtm(Qw)*W)', (sqrtm(Qy)*Y)';...
%         A{v}*W + B{v}*Y,    W                  , ZEROx        , ZEROxu;...
%         sqrtm(Qw)*W     , ZEROx              , Gamma*Ix     , ZEROxu;...
%         sqrtm(Qy)*Y     , ZEROux             ,ZEROux        , Gamma*Iu ] >= 0 ]:['Lmi_convx_v=',int2str(v)]];
%         Lmi_convix = Lmi_convix + lmi_conv_item;
%     end 
% 
% % LMI for input L2 norm       
%          Lmi_u_max = [[[ diag(umax) * diag(umax), Y;...
%              Y', W] >= 0 ]:['Lmi_maximum input constraints']];
% 
% % LMI for output
%      Lmi_output_max = [];
%      for v = 1 : nv
%          lmi_output_max_item = [[
%              [ W ,   (A{v}*W + B{v}*Y)'*C{v}' ;...
%              eye(ny)*(C{v}*(A{v}*W + B{v}*Y)), [(diag(ymax) * diag(ymax))+diag(s)] ] >= 0] :['Lmi_maximum output constraint']];  % diag(s)*10^5
%          Lmi_output_max = Lmi_output_max + lmi_output_max_item;
%      end
%     
%      for i=1:model1.pred.ny
%          con = con + [(0<=s(i,1)):['nonnegative_slacks_output_=',int2str(i)] ];
%      end
%     
% % Constraints
% con =  Lmi_Lyap +  Lmi_convix + Lmi_rie + Lmi_u_max + Lmi_output_max + con;
% 
%     % -------------  OBJECTIVE FUNCTION  -------------
%                          obj = Gamma;
% 
% %%   -------------  Optimizer  -------------
% 
%         options = sdpsettings('verbose', 1, 'solver','sedumi');       
%         mpc = optimizer(con,obj, options,{x,umax,ymax},{Y,W,Gamma});
% 
% 




x = zeros(model.pred.nx,1); %inital values
u = zeros(model.pred.nu,1);
ymaxred=300*ones(model.pred.ny,1);



D=dist.d(1:500,:)';    
ref=295*ones(model.pred.ny,1);
%ref=20;
for i=1:100
    d0=D(:,i);    
M   = inv([[model.pred.Ad - eye(model.pred.nx)], model.pred.Bd; model.pred.Cd, model.pred.Dd]);
M1  = M(1:model.pred.nx,:);
M2  = M(model.pred.nx+1:end,:);
Xss = M1 * [-(model.pred.Ed*d0 + model.pred.Gd); [ref - model.pred.Fd]];
Uss = M2 * [-(model.pred.Ed*d0 + model.pred.Gd); [ref - model.pred.Fd]];
%     M = inv([A-eye(nx) B; C 0]);
%     M1 = M(1:nx,:);
%     M2 = M(nx+1:end,:);
%     xss = M1*[-E*d(:,i); ref];
%     uss = M2*[-E*d(:,i); ref];
Xe = x - Xss;
Ue = u - Uss;
% if i==1 
%     Xe=zeros(model.pred.nx,1);
% end


umax = model.pred.umax - Uss;
ymax = ymaxred - model.pred.Cd*Xss + model.pred.Dd*Uss + model.pred.Fd; 

[solutions,info] = mpc{{Xe,umax,ymax}};
    Qnew = solutions{2};
    Ynew = solutions{1};
    gamma = solutions{3};
%    sett(i) = solutions{4};
    F = Ynew*inv(Qnew);
    u = Uss + F*Xe;
 
    x = model.pred.Ad*x + model.pred.Bd*u + model.pred.Ed*d0 + model.pred.Gd;
    XX(:,i)=x;
    y(:,i) = model.pred.Cd*x + model.pred.Dd*u + model.pred.Fd;

end
m=1
