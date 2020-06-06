
    % dimensions
    nx = 16;
    ny = 6;
    nd = 44;
    nu = 6;

    % horizons   
    N = 22;   %  prediction horizon
    Nc = 22; %  control horizon
    Nrp = 22; % reference preview horizon
    Ndp = 22; % disturbacne preview horizon

    % variables
    x = sdpvar(nx, N, 'full'); % states of the building
    d_prev = sdpvar(nd, Ndp, 'full'); % disturbances with preview
    % u = sdpvar(nu,   N, 'full'); % u = F * x
    % F = sdpvar(nu,nx*N, 'full'); % F = Y * W^-1
    W = sdpvar(nx,nx*N, 'full'); % F = Y * W^-1  where W > 0 and Y are obtained from the solution (if it exists) to the following linear objective minimization min Gamma
    Y = sdpvar(nu,nx*N, 'full'); % from the above equation.
    
    U_Cons = sdpvar(nu,nu*N);% variable to handle input constraints 
    Gamma = sdpvar(1,N);% minimize Gamma
    
    out = sdpvar(nx+nu,nx*N);% container for W and Y to get F 
    
    
    
    
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
    Qsb = 1e4*eye(ny);
    Qsa = 1e4*eye(ny);
    Qu = 1*eye(nu);
    Qx = 1*eye(nx);
    Qy = 1*eye(ny);
 

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
       


%% define F = YW^-1
%%then u = F*x


    for k = 1:N 
        
        %obj = x(:,k)'*Qx*x(:,k) + u(:,k)'*Qu*u(:,k) + Gamma(:,k);
        
        %F(:,k) = Y(:,1+(k-1)*nx:nx+(k-1)*nx) * (W(:,1+(k-1)*nx:nx+(k-1)*nx))^-1;
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

    Lmi_Lyap=[W(:,1+(k-1)*nx:nx+(k-1)*nx) >= 0];% W>=0
    Lmi_rie =[[1, x(:,k)'; x(:,k), W(:,1+(k-1)*nx:nx+(k-1)*nx)] >= 0 ];% condition to minimize gamma


% LMI for convix
    Lmi_convix = [];
    for v = 1 : nv
      lmi_conv_item = [ 
         [ W(:,1+(k-1)*nx:nx+(k-1)*nx)  ,  (A{v}*W(:,1+(k-1)*nx:nx+(k-1)*nx) + B{v}*Y(:,1+(k-1)*nx:nx+(k-1)*nx))' , (sqrt(Qx)*W(:,1+(k-1)*nx:nx+(k-1)*nx))', (sqrt(Qu)*Y(:,1+(k-1)*nx:nx+(k-1)*nx))';...
        A{v}*W(:,1+(k-1)*nx:nx+(k-1)*nx) + B{v}*Y(:,1+(k-1)*nx:nx+(k-1)*nx),    W(:,1+(k-1)*nx:nx+(k-1)*nx)                  , ZEROx        , ZEROxu;...
        sqrt(Qx)*W(:,1+(k-1)*nx:nx+(k-1)*nx)     , ZEROx              , Gamma(:,k)*Ix     , ZEROxu;...
        sqrt(Qu)*Y(:,1+(k-1)*nx:nx+(k-1)*nx)     , ZEROux             ,ZEROux        , Gamma(:,k)*Iu ] >= 0 ];

            Lmi_convix = Lmi_convix + lmi_conv_item;
    end 
%model.pred.umax =[10];
% LMI for input       
    Lmi_u_max = [];
    if(isempty(model.pred.umax) == 0)
        % L2-norm
        Lmi_u_max = [ [ diag(model.pred.umax.^2), Y(:,1+(k-1)*nx:nx+(k-1)*nx);...
            Y(:,1+(k-1)*nx:nx+(k-1)*nx)', W(:,1+(k-1)*nx:nx+(k-1)*nx)] >= 0 ];

        % L1-norm
        Lmi_u_max = Lmi_u_max + [ [ U_Cons(:,1+(k-1)*nu:nu+(k-1)*nu), Y(:,1+(k-1)*nx:nx+(k-1)*nx);...
            Y(:,1+(k-1)*nx:nx+(k-1)*nx)', W(:,1+(k-1)*nx:nx+(k-1)*nx)] >= 0 ];
        for j = 1 : nu
            Lmi_u_max = Lmi_u_max + [ U_Cons(j,j+(k-1)*nu) <= (model.pred.umax(j))^2 ];
        end 
    end 
        
    wm=[300;300;300;300;300;300];% Slack variable to be used% and % ymax 
% LMI for output
    Lmi_output_max = [];
    if(isempty(wa) == 0)
        for v = 1 : nv
            %%% idea to try for positive slack 
            %%%  con = con + [(0*ones(model.pred.ny,1)<=s(:,k)):['nonnegative_slacks_k=',int2str(k)] ]; 
            %%% 
        lmi_output_max_item = [ 
            [diag(wm.^2) , C{v}*(A{v}*W(:,1+(k-1)*nx:nx+(k-1)*nx) + B{v}*Y(:,1+(k-1)*nx:nx+(k-1)*nx));...
            (A{v}*W(:,1+(k-1)*nx:nx+(k-1)*nx) + B{v}*Y(:,1+(k-1)*nx:nx+(k-1)*nx))'*C{v}', W(:,1+(k-1)*nx:nx+(k-1)*nx) ] >= 0 ];% adding slack variable to the output y 
        
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
               % obj = obj + (model.pred.Cd * x(:,k)-r(:,k))'*Qy*(model.pred.Cd * x(:,k)-r(:,k)) + ...         %  comfort zone penalization
                            obj = obj + Gamma(:,k); %P*(uk'*Qu*uk)+...
                                                           %  quadratic penalization of ctrl action move blocking formulation
    end

        options = sdpsettings('verbose', 1, 'solver','gurobi');%,'gurobi.TimeLimit',50);
        
        sol = optimize(con,obj, options)
        solver = optimizer(con,obj, options,{x(:,1)},{W(:,1:nx),Y(:,1:nx)})
%% March 2020 
     % optimizer for dynamic comfort zone
     %if nd == 0  % no disturbances formulation
        % mpc = optimizer(con, obj, options,  { x(:, 1), wa_prev, wb_prev, price }, {u(:,1); obj} );
         %mpc = optimizer([con], obj, options,  { x(:, 1), wa_prev, wb_prev, price }, {u(:,1); obj} );

     %else
         %mpc = optimizer(con, obj, options,  { x(:, 1), d_prev, wa_prev, wb_prev, price }, out(:) );
         %mpc = optimizer([con], obj, options,  { x(:, 1), d_prev, wa_prev, wb_prev, price }, {u(:,1); obj} );
    % end    
