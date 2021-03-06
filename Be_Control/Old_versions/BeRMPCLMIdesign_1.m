function [mpc, constraints_info] = BeRMPCLMIdesign_1(model, RMPCLMIParam)
    %% MPC parameters

    % dimensions
    nx = model.pred.nx;
    ny = model.pred.ny;
    nd = model.pred.nd;
    nu = model.pred.nu;

    % variables
    x = sdpvar(nx, 1, 'full'); % states of the building
    %d_prev = sdpvar(nd, Ndp, 'full'); % disturbances with preview
    % u = sdpvar(nu,   N, 'full'); % u = F * x
    % F = sdpvar(nu,nx*N, 'full'); % F = Y * W^-1
    W = sdpvar(nx,nx, 'symmetric'); % F = Y * W^-1  where W > 0 and Y are obtained from the solution (if it exists) to the following linear objective minimization min Gamma
    %W = kron(ones(1,N),WW);
    Y = sdpvar(nu,nx, 'full'); % from the above equation.
    
    %U_Cons = sdpvar(nu,nu*N);% variable to handle input constraints 
    Gamma = sdpvar(1,1);% minimize Gamma
      
   %%
     %%
    s = sdpvar(ny, 1, 'full'); %  general slack
    y = sdpvar(ny, 1, 'full'); % output = indoor temperatures [degC]
    r = sdpvar(ny, 1, 'full'); %refernce
    slack1 = sdpvar(1,1);
    
    % above and below threshold -- dynamic comfort zone 
    wa_prev = sdpvar(ny, 1, 'full');
%    wb_prev = sdpvar(ny, Nrp, 'full');
    
    % variable energy price profile
    
    %price = sdpvar(1, Nrp, 'full');
    
    % weight diagonal matrices 
    Qsb = RMPCLMIParam.Qy;
    %Qsa = eye(ny);
    %Qy = 1e0*eye(nu);%-3 *e0RMPCLMIParam.Qy
    %Qw = 1e5*eye(nx);%RMPCLMIParam.Qw
    Qy = RMPCLMIParam.Qy;
    Qw = RMPCLMIParam.Qw;
    
    

    

     %% MPC problem formulation
    %  objective function+ constraints init
    obj = 0;
    con = [];
    
    %
    nv=3; % number of vertices 
    A{1,1} = 0.8 * model.pred.Ad;% +10% for all
    B{1,1} = 0.8 * model.pred.Bd;
    C{1,1} = 0.8 * model.pred.Cd;
    A{2,1} = 0.9 * model.pred.Ad;% -10% for all
    B{2,1} = 0.9 * model.pred.Bd;
    C{2,1} = 0.9 * model.pred.Cd;
    A{3,1} = model.pred.Ad;% nominal
    B{3,1} = model.pred.Bd;
    C{3,1} = model.pred.Cd;
    
    
    % to organise the matrices for LMI 
    ZEROx = zeros(nx,nx);
    ZEROux = zeros(nu,nx);
    ZEROxu = zeros(nx,nu);
    Ix = eye(nx);
    Iu = eye(nu);
 
    Qy = 1e-5*eye(nu);%-3 *e0
    Qw = 1e16*eye(nx);%C{3,1}'* C{3,1}*1e1 % -2 % 1e-2*eye(nu)
    Qsb= 1e2*eye(ny);
 for k = 1:1 



    Lmi_Lyap=[[W >= 0] : ['Lmi_Lyap_k=',int2str(k)]];% W>=0
    
    Lmi_rie =[[[1, x(:,k)'; x(:,k), W] >= 0] :['Lmi_rie_k=',int2str(k)] ];% condition to minimize gamma


% LMI for convix
    Lmi_convix = [];
    for v = 1 : nv
      Lmi_conv_item = [[ 
         [ W  ,  (A{v}*W + B{v}*Y)' , ((Qw)*W)', ((Qy)*Y)';...
        A{v}*W + B{v}*Y,    W                  , ZEROx        , ZEROxu;...
        (Qw)*W     , ZEROx              , Gamma*Ix     , ZEROxu;...
        (Qy)*Y     , ZEROux             ,ZEROux        , Gamma*Iu ] >= 0 ]:['Lmi_convx_k=',int2str(k)]];
    Lmi_convix = Lmi_convix + Lmi_conv_item;
    
    end 
    
% % m1 = [W W*A{1}'+Y'*B{1}' W*sqrtm(Qw) Y'*sqrtm(Qy)];
% % m2 = [A{1}*W+B{1}*Y, W, ZEROx ,ZEROxu];
% % m3 = [sqrtm(Qw)*W , ZEROx, Gamma*Ix  , ZEROxu];
% % m4 = [sqrtm(Qy)*Y , ZEROux  ,ZEROux   , Gamma*Iu];
% % L1 = [m1; m2; m3; m4];
% % 
% % 
% % m1 = [W W*A{2}'+Y'*B{2}' W*sqrtm(Qw) Y'*sqrtm(Qy)];
% % m2 = [A{2}*W+B{2}*Y, W, ZEROx ,ZEROxu];
% % m3 = [sqrtm(Qw)*W , ZEROx, Gamma*Ix  , ZEROxu];
% % m4 = [sqrtm(Qy)*Y , ZEROux  ,ZEROux   , Gamma*Iu];
% % L2 = [m1; m2; m3; m4];
% 
% 
% m1 = [W W*A{3}'+Y'*B{3}' W*sqrtm(Qw) Y'*sqrtm(Qy)];
% m2 = [A{3}*W+B{3}*Y, W, ZEROx ,ZEROxu];
% m3 = [sqrtm(Qw)*W , ZEROx, Gamma*Ix  , ZEROxu];
% m4 = [sqrtm(Qy)*Y , ZEROux  ,ZEROux   , Gamma*Iu];
% L3 = [m1; m2; m3; m4];

    
    
   
    
%             Lmi_convix = [L3>=0 ];%, L1>=0 , L2>=0];% + lmi_conv_item;
%     
    %con = con + [(0>=slack1)];

% LMI for input       
    Lmi_u_max = [];
    if(isempty(model.pred.umax) == 0)
        % L2-norm
        Lmi_u_max = [ [[ diag(model.pred.umax.^2), Y;...
            Y', W] >= 0 ]:['Lmi_L2_k=',int2str(k)]];
  end
        
    wm=[290;290;290;290;290;290];% Slack variable to be used% and % ymax 

    % LMI for output
    Lmi_output_max = [];
    if(isempty(wm) == 0)
        
        
        %con = con + [(0*ones(model.pred.ny,1)<=s):['nonnegative_slacks_k=',int2str(k)] ]; 
         for(i=1:6)
         con = con + [(0<=s(i,1)):['nonnegative_slacks_k=',int2str(i)] ]; 
         end
        
        
        for v = 1 : nv
           
        lmi_output_max_item = [[ 
            [ W,(A{v}*W + B{v}*Y)'*C{v}';...
             C{v}*(A{v}*W + B{v}*Y), diag((wm.^2)+s) ] >= 0] :['Lmi_Y_k=',int2str(k)]  ];% adding slack variable to the output y 
        
        Lmi_output_max = Lmi_output_max + lmi_output_max_item;
        end 
    end
    
    
% Constraints
con = con + Lmi_Lyap + Lmi_rie + Lmi_convix + Lmi_u_max + Lmi_output_max;
   
    %   -------------  OBJECTIVE FUNCTION  -------------
                           % obj = obj  + Gamma; 
                            
                            
                            obj = obj +(r - C{1,1}*x )' * Qsb * (r - C{1,1}*x);
                            obj = obj +(r - C{2,1}*x )' * Qsb * (r - C{2,1}*x);
                            obj = obj +(r - C{3,1}*x )' * Qsb * (r - C{3,1}*x);
                            
    end

        opts = sdpsettings('verbose', 1, 'solver','mosek');%,'gurobi.TimeLimit',50); 
        sol = optimize(con,obj, opts);
        solver = optimizer(con,obj, opts,{x(:,1),r},{Y(:,1:nx),W(:,1:nx)});
        mpc=solver;

%      information about constraints size and type
     constraints_info.con = con;
     constraints_info.size = size(con);
     constraints_info.i_length = NaN(length(con),1);
     constraints_info.equality = is(con,'equality'); 
        
    for k = 1:length(con) 
        constraints_info.i_length(k) = length(double(con(k)));       
    end

         output=solver{{zeros(20,1),[296.65;296.65;296.65;296.65;296.65;296.65]}};
         yy=output{1,1};
         ww=output{1,2};
%        gg=output{1,3};
%        SSS=output{1,4};
         F=yy*ww^-1
end