function [mpc, constraints_info] = BeRMPCLMIdesign(model, RMPCLMIParam)
    %% MPC parameters

    % dimensions
    nx = model.pred.nx;
    ny = model.pred.ny;
    nd = model.pred.nd;
    nu = model.pred.nu;

    % horizons    it is consider as infinite Horizon with LMI 
    N = RMPCLMIParam.N;   %  prediction horizon
    Nc = RMPCLMIParam.Nc; %  control horizon
    Nrp = RMPCLMIParam.Nrp; % reference preview horizon
    Ndp = RMPCLMIParam.Ndp; % disturbacne preview horizon

   % variables
    x = sdpvar(nx, 1, 'full'); % states of the building
    d_prev = sdpvar(nd, Ndp, 'full'); % disturbances with preview
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
    y = sdpvar(ny, N, 'full'); % output = indoor temperatures [degC]
    r = sdpvar(ny, Nrp, 'full'); %refernce
    slack1 = sdpvar(1,1);
    
    % above and below threshold -- dynamic comfort zone 
    wa_prev = sdpvar(ny, 1, 'full');
    wb_prev = sdpvar(ny, Nrp, 'full');
    
    % variable energy price profile
    
    price = sdpvar(1, Nrp, 'full');
    
    % weight diagonal matrices 
    %Qsb = eye(ny);
    %Qsa = eye(ny);
    %Qy = 1e0*eye(nu);%-3 *e0RMPCLMIParam.Qy
    %Qw = 1e5*eye(nx);%RMPCLMIParam.Qw
    Qy = RMPCLMIParam.Qy;
    Qw = RMPCLMIParam.Qw;% look down \|/
    
    

     %% MPC problem formulation
    %  objective function+ constraints init
    obj = 0;
    con = [];
    
    %
    nv=3; % number of vertices 
    A{1,1} = 1.1 * model.pred.Ad;% +10% for all
    B{1,1} = 1.1 * model.pred.Bd;
    C{1,1} = 1.1 * model.pred.Cd;
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
 
    
    
    
for k = 1:1 

        %   -------------  Constraints  -------------
 if k > Nrp
            wa = wa_prev(:,Nrp);
            wb = wb_prev(:,Nrp);
            P = price(:,Nrp);
        else
            wa = wa_prev(:,k);
            wb = wb_prev(:,k);
            P = price(:,k);
        end

    Lmi_Lyap=[[W >= 0] : ['Lmi_Lyap_k=',int2str(k)]];% W>=0
    
    Lmi_rie =[[[1, x(:,k)'; x(:,k), W] >= 0] :['Lmi_rie_k=',int2str(k)] ];% condition to minimize gamma


% LMI for convix
    Lmi_convix = [];
    for v = 1 : nv
      lmi_conv_item = [[ 
         [ W  ,  (A{v}*W + B{v}*Y)' , (sqrt(Qw)*W)', (sqrt(Qy)*Y)';...
        A{v}*W + B{v}*Y,    W                  , ZEROx        , ZEROxu;...
        sqrt(Qw)*W     , ZEROx              , Gamma*Ix     , ZEROxu;...
        sqrt(Qy)*Y     , ZEROux             ,ZEROux        , Gamma*Iu ] >= slack1 ]:['Lmi_convx_k=',int2str(k)]];
            Lmi_convix = Lmi_convix + lmi_conv_item;
    end 

% LMI for input       
    Lmi_u_max = [];
    if(isempty(model.pred.umax) == 0)
        % L2-norm
        Lmi_u_max = [ [[ diag(model.pred.umax.^2), Y;...
            Y', W] >= 0 ]:['Lmi_L2_k=',int2str(k)]];
  end
        
    wm=[280;280;280;280;280;280];% Slack variable to be used% and % ymax 

    % LMI for output
    Lmi_output_max = [];
    if(isempty(wa) == 0)
        con = con + [(0*ones(model.pred.ny,1)<=s):['nonnegative_slacks_k=',int2str(k)] ]; 
        con = con + [(0>=slack1):['negative_slack1_k=',int2str(k)] ]; 

        for v = 1 : nv
           
        lmi_output_max_item = [[ 
            [ W,(A{v}*W + B{v}*Y)'*C{v}' ;...
            C{v}*(A{v}*W + B{v}*Y), diag((wm.^2)+s) ] >= 0] :['Lmi_Y_k=',int2str(k)]  ];% adding slack variable to the output y 
        
        Lmi_output_max = Lmi_output_max + lmi_output_max_item;
        end 
    end
    
    
% Constraints
con = con + Lmi_Lyap + Lmi_rie + Lmi_convix + Lmi_u_max + Lmi_output_max;
   
    %   -------------  OBJECTIVE FUNCTION  -------------
                            obj = obj  + Gamma; 
end


        options = sdpsettings('verbose', 1, 'solver','sedumi');%,'gurobi.TimeLimit',50);
        
  %      sol = optimize(con,obj, options);
        mpc = optimizer(con,obj, options,{x(:,1)},{Y(:,1:nx),W(:,1:nx)});
             

%      information about constraints size and type
     constraints_info.con = con;
     constraints_info.size = size(con);
     constraints_info.i_length = NaN(length(con),1);
     constraints_info.equality = is(con,'equality'); 
        
    for k = 1:length(con) 
        constraints_info.i_length(k) = length(double(con(k)));       
    end


end