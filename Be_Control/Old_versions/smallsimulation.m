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
model1=model_Old;
model2=model_Reno;
model3=model_Light;


mod = model_Old;      % construct a model for prediction   

%% Disturbacnes 
% ambient temperature, solar radiation, internal heat gains
DistParam.reload = 0;
dis = BeDist(mod, DistParam);
%mpc=BeRMPCLMI_test(model_Old,model_Reno,model_Light);
% comfort constraints, price profiles
RefsParam.Price.variable = 0;       %1 =  variable price profile, 0 = fixed to 1

refs = BeRefs(model_Reno, dis, RefsParam);     % construct a references object  


% controller.ARMPC.N = 22;%3;%22;
% controller.ARMPC.Nc = 22;%22;
% controller.ARMPC.Nrp = 22;%22;
% controller.ARMPC.Ndp = 22;%22;
% % weight diagonal matrices
% controller.ARMPC.Qsb = 1e8*eye(model_Old.pred.ny);
% controller.ARMPC.Qsa = 1e8*eye(model_Old.pred.ny);
% controller.ARMPC.Qu  = 1e2*eye(model_Old.pred.nu);
% controller.ARMPC.use=    1; % L2020 RMPC MUP Use 
% controller.ARMPC.Condensing =    1;
%     
% mpc_1=BeRMPCMINMAX(model_Old, controller.ARMPC);
% mpc_2=BeRMPCMINMAX(model_Reno, controller.ARMPC);
% mpc_3=BeRMPCMINMAX(model_Light, controller.ARMPC);


%% ----------------- 

% 
% 
 %% MPC parameters
    % dimensions
    nx = model1.pred.nx;
    ny = model1.pred.ny;
    nd = model1.pred.nd;
    nu = model1.pred.nu;

    % variables
    x = sdpvar(nx, 1, 'full'); % states of the building
    W = sdpvar(nx,nx, 'symmetric'); % F = Y * W^-1  where W > 0 and Y are obtained from the solution (if it exists) to the following linear objective minimization min Gamma
    Y = sdpvar(nu,nx, 'full'); % from the above equation.
    umax = sdpvar(nu,1);
    ymax = sdpvar(ny,1);
    Gamma = sdpvar(1,1);% minimize Gamma or (1,1)
    s = sdpvar(ny,1); % maximum output constraint slack.
      

    
    %% Models
    %  objective function+ constraints init
    obj = 0;
    con = [];
    %
    nv=3; % number of vertices 
    A{1,1} =  model1.pred.Ad;
    B{1,1} =  model1.pred.Bd;
    C{1,1} =  model1.pred.Cd;
    A{2,1} =  model2.pred.Ad;
    B{2,1} =  model2.pred.Bd;
    C{2,1} =  model2.pred.Cd;
    A{3,1} =  model3.pred.Ad;
    B{3,1} =  model3.pred.Bd;
    C{3,1} =  model3.pred.Cd;
    
   %% Weight Matrices
   
    Qy = eye(nu)*10^-18;%-3%-3
    Qw = C{1,1}'*C{1,1}*10^0;%0%1makes the line away-1
    %Qw = Qx'*Qx*10^5;
 
    % to organise the matrices for LMI 
    ZEROx = zeros(nx,nx);
    ZEROux = zeros(nu,nx);
    ZEROxu = zeros(nx,nu);
    Ix = eye(nx);
    Iu = eye(nu);

        %   -------------  Constraints  -------------
    Lmi_Lyap=[[W >= 0] : ['Constraint on W']];
    Lmi_rie =[[[1, x'; x, W] >= 0] :['constraint on X'] ];

% LMI for convix
    Lmi_convix = [];
    for v = 1 : nv
      lmi_conv_item = [[ 
         [ W  ,  (A{v}*W + B{v}*Y)' , (sqrtm(Qw)*W)', (sqrtm(Qy)*Y)';...
        A{v}*W + B{v}*Y,    W                  , ZEROx        , ZEROxu;...
        sqrtm(Qw)*W     , ZEROx              , Gamma*Ix     , ZEROxu;...
        sqrtm(Qy)*Y     , ZEROux             ,ZEROux        , Gamma*Iu ] >= 0 ]:['Lmi_convx_v=',int2str(v)]];
        Lmi_convix = Lmi_convix + lmi_conv_item;
    end 

% LMI for input L2 norm       
         Lmi_u_max = [[[ diag(umax) * diag(umax), Y;...
             Y', W] >= 0 ]:['Lmi_maximum input constraints']];

% LMI for output
     Lmi_output_max = [];
     for v = 1 : nv
         lmi_output_max_item = [[
             [ W ,   (A{v}*W + B{v}*Y)'*C{v}' ;...
             eye(ny)*(C{v}*(A{v}*W + B{v}*Y)), [(diag(ymax) * diag(ymax))+diag(s)] ] >= 0] :['Lmi_maximum output constraint']];  % diag(s)*10^5
         Lmi_output_max = Lmi_output_max + lmi_output_max_item;
     end
    
     for i=1:model1.pred.ny
         con = con + [(0<=s(i,1)):['nonnegative_slacks_output_=',int2str(i)] ];
     end
    
% Constraints
con =  Lmi_Lyap +  Lmi_convix + Lmi_rie + Lmi_u_max + Lmi_output_max + con;

    % -------------  OBJECTIVE FUNCTION  -------------
                         obj = Gamma;

%%   -------------  Optimizer  -------------

        options = sdpsettings('verbose', 1, 'solver','mosek');       
        mpc = optimizer(con,obj, options,{x,umax,ymax},{Y,W,Gamma});






x = zeros(mod.pred.nx,1); %inital values
u = zeros(mod.pred.nu,1);
ymaxred=300*ones(mod.pred.ny,1);


SimStart_sec = (200-1)*86400;
SimStop_sec = (200)*86400;
% starting and finishing  step for simulation loop - MPC
SimStart = floor(SimStart_sec/900)+1;
SimStop = ceil(SimStop_sec/900);
Nsim = length(SimStart:SimStop);
N=22;
wa = refs.wa(SimStart:SimStop+N,:)';
wb = refs.wb(SimStart:SimStop+N,:)';
    % ------ PMV zone ------ 

%D=dis.d(SimStart:SimStop,:)';   

D = dis.d(SimStart:SimStop+22,:)';
%D(41,:) = dis.d(SimStart:SimStop,41)';  
%D=dis.d(1:500,:)';    
ref=297*ones(mod.pred.ny,1);
%ref=20;
for i=1:Nsim
    d0=D(:,i); 
    
    
M   = inv([[mod.pred.Ad - eye(mod.pred.nx)], mod.pred.Bd; mod.pred.Cd, mod.pred.Dd]);
M1  = M(1:mod.pred.nx,:);
M2  = M(mod.pred.nx+1:end,:);
Xss = M1 * [-(mod.pred.Ed*d0 + mod.pred.Gd); [ref - mod.pred.Fd]];
Uss = M2 * [-(mod.pred.Ed*d0 + mod.pred.Gd); [ref - mod.pred.Fd]];
ss(:,i) =Uss; 
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


umax = mod.pred.umax - Uss;
ymax = ymaxred -(mod.pred.Cd*Xss + mod.pred.Dd*Uss + mod.pred.Fd); 
err(:,i)=Xe;
[solutions,info] = mpc{{Xe,umax,ymax}};
    Qnew = solutions{2};
    Ynew = solutions{1};
    gamma = solutions{3};
%    sett(i) = solutions{4};
    F = Ynew * inv(Qnew);
    u = Uss + F*Xe;
    for jj=1:6
        if u(jj,:)<0
            u(jj,:)=0;
        end
    end
    U(:,i)=u;
    x = mod.pred.Ad*x + mod.pred.Bd*u + mod.pred.Ed*d0 + mod.pred.Gd;
    XX(:,i)=x;
    y(:,i) = mod.pred.Cd*x + mod.pred.Dd*u + mod.pred.Fd;
   if (rem(i,96)==0)
       i
   end

end
m=1
tt=1:Nsim;

figure
subplot(2,1,1)
plot(tt,y-273.15)
hold on
Rmin = mean(wb(:,1:end-N),1);
Rmax = mean(wa(:,1:end-N),1);
%          R = outdata.data.R;
stairs(tt, Rmin-273.15, 'k--', 'linewidth', 2);
stairs(tt, Rmax-273.15, 'k--', 'linewidth', 2);
subplot(2,1,2)
plot(tt,U)
figure
plot(tt,err)



