    % dimensions
    nx = 30;
    ny = 6;
    nd = 44;
    nu = 6;

    % horizons   
    N = 2;   %  prediction horizon
    Nc = 22; %  control horizon
    Nrp = 22; % reference preview horizon
    Ndp = 22; % disturbacne preview horizon

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
   
    %Qy = 1e4*eye(ny);
 

    %% MPC problem formulation
    %  objective function+ constraints init
    obj = 0;
    con = [];
    ZERO=0;
    
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

    
    Qy = 1e9*eye(nu);%-3 *e0
    Qw = C{3,1}'* C{3,1}*1e5;%C{3,1}'* C{3,1}*1e1 % -2 % 1e-2*eye(nu)
    % sqrt qw will go to 10^-3 
    %1e0*eye(nx)
    %C{3,1}'* C{3,1}*1e5; perfect mosek 
    
    % to organise the matrices for LMI 
    ZEROx = zeros(nx,nx);
    ZEROux = zeros(nu,nx);
    ZEROxu = zeros(nx,nu);
    Ix = eye(nx);
    Iu = eye(nu);
% comfort zone and price preview 



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

    Lmi_Lyap=[[W >= ZERO] : ['Lmi_Lyap_k=',int2str(k)]];% W>=0
    
    Lmi_rie =[[[1, x(:,k)'; x(:,k), W] >= ZERO] :['Lmi_rie_k=',int2str(k)] ];% condition to minimize gamma


% LMI for convix
    Lmi_convix = [];
    for v = 1 : nv
      lmi_conv_item = [[ 
         [ W  ,  (A{v}*W + B{v}*Y)' , ((Qw)*W)', ((Qy)*Y)';...
        A{v}*W + B{v}*Y,    W                  , ZEROx        , ZEROxu;...
        (Qw)*W     , ZEROx              , Gamma*Ix     , ZEROxu;...
        (Qy)*Y     , ZEROux             ,ZEROux        , Gamma*Iu ] >= slack1 ]:['Lmi_convx_k=',int2str(k)]];
            Lmi_convix = Lmi_convix + lmi_conv_item;
    end 
    con = con + [(0>=slack1)];

% LMI for input       
    Lmi_u_max = [];
    if(isempty(model.pred.umax) == 0)
        % L2-norm
        Lmi_u_max = [ [[ diag(model.pred.umax.^2), Y;...
            Y', W] >= ZERO ]:['Lmi_L2_k=',int2str(k)]];
  end
        
    wm=[297;297;297;297;297;297];% Slack variable to be used% and % ymax 

    % LMI for output
    Lmi_output_max = [];
    if(isempty(wa) == 0)
        con = con + [(0*ones(model.pred.ny,1)<=s):['nonnegative_slacks_k=',int2str(k)] ]; 
        for v = 1 : nv
           
        lmi_output_max_item = [[ 
            [ W,(A{v}*W + B{v}*Y)'*C{v}';...
             C{v}*(A{v}*W + B{v}*Y), diag((wm.^2)+s) ] >= ZERO] :['Lmi_Y_k=',int2str(k)]  ];% adding slack variable to the output y 
        
        Lmi_output_max = Lmi_output_max + lmi_output_max_item;
        end 
    end
    
    
% Constraints
con = con + Lmi_Lyap + Lmi_rie + Lmi_convix + Lmi_u_max + Lmi_output_max;
   
    %   -------------  OBJECTIVE FUNCTION  -------------
                            obj = obj  + Gamma; 
    end

        options = sdpsettings('verbose', 1, 'solver','mosek');%,'gurobi.TimeLimit',50);
        
        sol = optimize(con,obj, options);
        solver = optimizer(con,obj, options,{x(:,1)},{Y(:,1:nx),W(:,1:nx),Gamma,slack1});
        mpc=solver;
        
        
        
        
        
         output=solver(zeros(30,1));
         yy=output{1,1};
         ww=output{1,2};
         gg=output{1,3};
         SSS=output{1,4};
         F=yy*ww^-1
         value (slack1)
         
         
         
         vv= [       -0.1360
    0.0154
    0.1718
   -0.1434
   -0.3902
   -0.4856
   -0.2083
    0.0990
   -0.2466
    0.7996
   -0.6544
    0.7376
    3.7341
    2.0101
   -0.2900
   -0.8969
   -1.3846
    0.2615
   -3.0661
    0.2339
   -1.4442
   -0.3033
   -1.0581
    1.2638
   -0.6809
    0.2688
   -0.9613
   -1.8422
   -0.2674
    0.8394];
% 
    output1=solver(vv);

         yy1=output1{1,1};
         ww1=output1{1,2};
         gg1=output{1,3};
         SSS1=output{1,4};
         
         F1=yy1*ww1^-1
         
         
  

