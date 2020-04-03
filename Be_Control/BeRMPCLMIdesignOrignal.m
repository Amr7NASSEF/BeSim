function [mpc, constraints_info] = BeMPCdesign(model, RMPCLMIParam)
% MPC design function using Yalmip
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
   RMPCLMIParam.use = 0;
   RMPCLMIParam.Condensing = 1;
   % horizons
   RMPCLMIParam.N = 2;
   RMPCLMIParam.Nc = 2;
   RMPCLMIParam.Nrp = 2;
   RMPCLMIParam.Ndp = 2;
   % weight diagonal matrices 
   RMPCLMIParam.Qsb = 1e6*eye(model.pred.ny);
   RMPCLMIParam.Qsa = 1e6*eye(model.pred.ny);
   RMPCLMIParam.Qu = 1e0*eye(model.pred.nu);
end
    %% MPC parameters

    % dimensions
    nx = model.pred.nx;
    ny = model.pred.ny;
    nd = model.pred.nd;
    nu = model.pred.nu;

    % horizons   
    N = RMPCLMIParam.N;   %  prediction horizon
    Nc = RMPCLMIParam.Nc; %  control horizon
    Nrp = RMPCLMIParam.Nrp; % reference preview horizon
    Ndp = RMPCLMIParam.Ndp; % disturbacne preview horizon

    % variables
    x = sdpvar(nx, N+1, 'full'); % states of the building
    d_prev = sdpvar(nd, Ndp, 'full'); % disturbances with preview

    W = sdpvar(nx,nx*N, 'full'); % F = Y * W^-1  where W > 0 and Y are obtained from the solution (if it exists) to the following linear objective minimization min Gamma
    Y = sdpvar(nu,nx*N, 'full'); % from the above equation.
    
    U_Cons = sdpvar(nu,nu*N);% variable to handle input constraints 
    Gamma = sdpvar(1,N);% minimize Gamma
    
    %out = sdpvar(nx+nu,nx*N);% container for W and Y to get F 
    
    
    
    
% u = sdpvar(nu, Nc, 'full'); % ctrl action - heat commanded b;y the thermostat [W]
   %%
    s = sdpvar(ny, N, 'full'); %  general slack
    y = sdpvar(ny, N, 'full'); % output = indoor temperatures [degC]
    r = sdpvar(ny, Nrp, 'full'); %refernce
   
    % above and below threshold -- dynamic comfort zone 
    wa_prev = sdpvar(ny, Nrp, 'full');
    wb_prev = sdpvar(ny, Nrp, 'full');
    
    % variable energy price profile
    
    price = sdpvar(1, Nrp, 'full');
    
    % weight diagonal matrices 
    Qsb = RMPCLMIParam.Qsb;
    Qsa = RMPCLMIParam.Qsa;
    Qu = RMPCLMIParam.Qu;
    Qx = RMPCLMIParam.Qx;
    Qy = RMPCLMIParam.Qy;

    %% MPC problem formulation
    %  objective function+ constraints init
    obj = 0;
    con = [];
    ZERO=1e-6;
    
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
% comfort zone and price preview 
       

    for k = 1:N 
        
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

% LMI for minimizing gamma
    Lmi_Lyap=[W(:,1+(k-1)*nx:nx+(k-1)*nx) >= ZERO];% W>=0
    Lmi_rie =[[1, x(:,k)'; x(:,k), W(:,1+(k-1)*nx:nx+(k-1)*nx)] >= ZERO ];% condition to minimize gamma


% LMI for convix
    Lmi_convix = [];
    for v = 1 : nv
      lmi_conv_item = [ 
         [ W(:,1+(k-1)*nx:nx+(k-1)*nx)  ,  (A{v}*W(:,1+(k-1)*nx:nx+(k-1)*nx) + B{v}*Y(:,1+(k-1)*nx:nx+(k-1)*nx))' , (sqrt(Qx)*W(:,1+(k-1)*nx:nx+(k-1)*nx))', (sqrt(Qu)*Y(:,1+(k-1)*nx:nx+(k-1)*nx))';...
        A{v}*W(:,1+(k-1)*nx:nx+(k-1)*nx) + B{v}*Y(:,1+(k-1)*nx:nx+(k-1)*nx),    W(:,1+(k-1)*nx:nx+(k-1)*nx)                  , ZEROx        , ZEROxu;...
        sqrt(Qx)*W(:,1+(k-1)*nx:nx+(k-1)*nx)     , ZEROx              , Gamma(:,k)*Ix     , ZEROxu;...
        sqrt(Qu)*Y(:,1+(k-1)*nx:nx+(k-1)*nx)     , ZEROux             ,ZEROux        , Gamma(:,k)*Iu ] >= ZERO ];

            Lmi_convix = Lmi_convix + lmi_conv_item;
    end 

% LMI for input       
    Lmi_u_max = [];
    if(isempty(model.pred.umax) == 0)
        % L2-norm
        Lmi_u_max = [ [ diag(model.pred.umax.^2), Y(:,1+(k-1)*nx:nx+(k-1)*nx);...
            Y(:,1+(k-1)*nx:nx+(k-1)*nx)', W(:,1+(k-1)*nx:nx+(k-1)*nx)] >= ZERO ];

        % L1-norm
        Lmi_u_max = Lmi_u_max + [ [ U_Cons(:,1+(k-1)*nu:nu+(k-1)*nu), Y(:,1+(k-1)*nx:nx+(k-1)*nx);...
            Y(:,1+(k-1)*nx:nx+(k-1)*nx)', W(:,1+(k-1)*nx:nx+(k-1)*nx)] >= ZERO ];
        for j = 1 : nu
            Lmi_u_max = Lmi_u_max + [ U_Cons(j,j+(k-1)*nu) <= (model.pred.umax(j))^2 ];
        end 
    end 
        
    wm=[300;300;300;300;300;300];% Slack variable to be used% and % ymax =wm
% LMI for output
    Lmi_output_max = [];
    if(isempty(wa) == 0)
        for v = 1 : nv
            %%% idea to try for positive slack 
            %%%  con = con + [(0*ones(model.pred.ny,1)<=s(:,k)):['nonnegative_slacks_k=',int2str(k)] ]; 
            %%% 
        lmi_output_max_item = [ 
            [diag(wm.^2) , C{v}*(A{v}*W(:,1+(k-1)*nx:nx+(k-1)*nx) + B{v}*Y(:,1+(k-1)*nx:nx+(k-1)*nx));...
            (A{v}*W(:,1+(k-1)*nx:nx+(k-1)*nx) + B{v}*Y(:,1+(k-1)*nx:nx+(k-1)*nx))'*C{v}', W(:,1+(k-1)*nx:nx+(k-1)*nx) ] >= ZERO ];% adding slack variable to the output y 
        
        Lmi_output_max = Lmi_output_max + lmi_output_max_item;
        end 
    end
    
    
% to organise the Optimizer Matrices as we have more than 1 decision
% variable 


% row = [nx,nu,1];
% col = [nx,nx,1];
% %
% if(isempty(model.pred.umax) == 0) % For Input Constraints U_MAX
%     row = [row,nu];
% 	col = [col,nu];
% end % if
% 
% Wt = [W(:,1+(k-1)*nx:nx+(k-1)*nx), zeros(row(1),max(col)-col(1))];
% Yt = [Y(:,1+(k-1)*nx:nx+(k-1)*nx), zeros(row(2),max(col)-col(2))];
% gt = [Gamma(:,k), zeros(row(3),max(col)-col(3))];
% M(:,1+(k-1)*nx:nx+nx*(k-1))  = [Wt; Yt];
% % if(isempty(model.pred.umax) == 0) % For Input Constraints U_MAX
% %     Ut = [U_Cons(:,1+(k-1)*nu:nu+(k-1)*nu), zeros(row(4),max(col)-col(4))];
% %     M  = [M; Ut];
% % end % if
% %


% Constraints
con = con + Lmi_Lyap + Lmi_rie + Lmi_convix + Lmi_u_max + Lmi_output_max;%  + [out(:,1+(k-1)*nx:nx+nx*(k-1)) == M(:,1+(k-1)*nx:nx+nx*(k-1))];%+ Lmi_output_max

    

    
    
      
    %   -------------  OBJECTIVE FUNCTION  -------------
        %    % quadratic objective function withouth states constr.  penalisation
                %obj = obj + (model.pred.Cd * x(:,k)-r(:,k))'*Qy*(model.pred.Cd * x(:,k)-r(:,k)) + ...         %  comfort zone penalization
                    obj = obj +Gamma(:,k); %P*(uk'*Qu*uk)+...
                                                           %  quadratic penalization of ctrl action move blocking formulation
    end


     %% construction of object optimizer
     %   structure:  optimizer(constraints, objecttive, options, input_params, output_params)

    %  optimizer options
   % try
        options = sdpsettings('verbose', 1, 'solver','gurobi');%,'gurobi.TimeLimit',50);
         sol = optimize(con,obj, options)
         
         mpc = optimizer(con, obj, options,  { x(:, 1), d_prev, wa_prev, wb_prev, price }, {Y(:,1:nx),W(:,1:nx)} );
         mpc = optimizer(con, obj, options,  { x(:, 1) }, {Y(:,1:nx),W(:,1:nx)} );
         %mpc = optimizer([con], obj, options,  { x(:, 1), d_prev, wa_prev, wb_prev, price }, {u(:,1); obj} );
    % end    

%      information about constraints size and type
     constraints_info.con = con;
     constraints_info.size = size(con);
     constraints_info.i_length = NaN(length(con),1);
     constraints_info.equality = is(con,'equality'); 
        
    for k = 1:length(con) 
        constraints_info.i_length(k) = length(double(con(k)));       
    end


end