function [mpc, constraints_info] = BeAMPCdesign(model, AMPCParam)
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
   AMPCParam.use = 0;
   AMPCParam.Condensing = 1;
   % horizons
   AMPCParam.N = 22;
   AMPCParam.Nc = 22;
   AMPCParam.Nrp = 22;
   AMPCParam.Ndp = 22;
   % weight diagonal matrices 
   AMPCParam.Qsb = 1e6*eye(model.pred.ny);
   AMPCParam.Qsa = 1e6*eye(model.pred.ny);
   AMPCParam.Qu = 1e0*eye(model.pred.nu);
end

    %% MPC parameters

    % dimensions
    nx = model.pred.nx;
    ny = model.pred.ny;
    nd = model.pred.nd;
    nu = model.pred.nu;

    % horizons   
    N = AMPCParam.N;   %  prediction horizon
    Nc = AMPCParam.Nc; %  control horizon
    Nrp = AMPCParam.Nrp; % reference preview horizon
    Ndp = AMPCParam.Ndp; % disturbacne preview horizon

    % variables
    x = sdpvar(nx, N+1, 'full'); % states of the building
    d_prev = sdpvar(nd, Ndp, 'full'); % disturbances with preview
    u = sdpvar(nu, Nc, 'full'); % ctrl action - heat commanded by the thermostat [W]
    y = sdpvar(ny, N, 'full'); % output = indoor temperatures [degC]
    s = sdpvar(ny, N, 'full'); %  general slack
    A = sdpvar(nx,nx,'full');
    B = sdpvar(nx,nu,'full');
%     AB = sdpvar( nx , N*nu ,'full');
%     AE = sdpvar( nx , N*nd,'full' );
%     AG = sdpvar( nx, N*1,'full');
%     % above and below threshold -- dynamic comfort zone 
    wa_prev = sdpvar(ny, Nrp, 'full');
    wb_prev = sdpvar(ny, Nrp, 'full');
    % variable energy price profile
    price = sdpvar(1, Nrp, 'full');
    
%     u_traj = sdpvar(nu, Nc, 'full');
    
    % weight diagonal matrices 
    Qsb = AMPCParam.Qsb;
    Qsa = AMPCParam.Qsa;
    Qu = AMPCParam.Qu;

    %% MPC problem formulation
    %  objective function+ constraints init
    obj = 0;
    con = [];
    
    AB = zeros( nx , N*nu );
    AE = zeros( nx , N*nd );
    AG = zeros( nx, N*1);


    AExpX0 = eye(nx) * x(:,1);
   
    for k = 1:N   
    %   -------------  Constraints  -------------
        % disturbances preview
        if k > Ndp
            Dpreview = d_prev(:, Ndp);
        else
            Dpreview = d_prev(:, k);
        end

            % comfort zone and price preview 
        if k > Nrp
            wa = wa_prev(:,Nrp);
            wb = wb_prev(:,Nrp);
            P = price(:,Nrp);
        else
            wa = wa_prev(:,k);
            wb = wb_prev(:,k);
            P = price(:,k);
        end

            % move blocking
        if k > Nc
            uk = u(:,Nc);
%             u_traj(:,k) = u(:,Nc);
        else
            uk = u(:,k);
%             u_traj(:,k) = u(:,k);
        end

          %     state + output update equations
        if AMPCParam.Condensing %  Single shooting
            if k == 1
                AB = [AB(:,1: (N-k)*nu ) ,B];        %  input matrix evolution
               %con = con+ [AB(:, (N-k)*nu+1:(N-k+1)*nu ) == B];        %  input matrix evolution
                AE(:, (N-k)*nd+1:(N-k+1)*nd ) =  model.pred.Ed;       %  disturbance matrix evolution
                AG(:, (N-k)*1+1:(N-k+1)*1 ) =  model.pred.Gd;         %  initial conditions matrix evolution
                con = con + [ (y(:, k) == model.pred.Cd*x(:, k) + model.pred.Dd*uk + model.pred.Fd*1):['SSM_single_shoot_k=',int2str(k)] ];
            else
                AExpX0 = A * AExpX0;
                con = con + [ (y(:, k) == model.pred.Cd*( AExpX0 + AB(:, (N-k+1)*nu+1 : end ) * reshape( u(:,1:k-1) , nu * (k-1) , 1) + ...
                                                                 AE(:, (N-k+1)*nd+1 : end ) * reshape( d_prev(:,1:k-1) , nd * (k-1) , 1) + ...
                                                                 AG(:, (N-k+1)*1+1 : end ) * ones(k-1,1) ) + ...
                                                                 model.pred.Dd*uk  + model.pred.Fd*1):['SSM_single_shoot_k=',int2str(k)] ];

                AB(:, (N-k)*nu+1:(N-k+1)*nu ) = A* AB(:, (N-k+1)*nu+1:(N-k+2)*nu );
               % AE(:, (N-k)*nd+1:(N-k+1)*nd ) = A* AE(:, (N-k+1)*nd+1:(N-k+2)*nd );
                AE = [AE(:,1:(N-k)*nd), A* AE(:, (N-k+1)*nd+1:(N-k+2)*nd ), AE(:,(N-k)*nd+1+nd :end)];
                AG = [AG(:,1:(N-k)*1), A* AG(:, (N-k+1)*1+1:(N-k+2)*1 ), AG(:,(N-k)*1+1+1 :end)];
              %  AG(:, (N-k)*1+1:(N-k+1)*1 ) = A* AG(:, (N-k+1)*1+1:(N-k+2)*1 );
            end  
        else %  Multiple shooting
            if nd == 0 % no disturbances formulation
                con = con + [ (x(:, k+1) == A*x(:, k) + B*uk + model.pred.Gd*1):['SSM_multiple_shoot_states_k=',int2str(k)] ];
                con = con + [ (y(:, k) == model.pred.Cd*x(:, k)  + model.pred.Dd*uk + model.pred.Fd*1):['SSM_multiple_shoot_outputs_k=',int2str(k)] ];
            else
                con = con + [ (x(:, k+1) == A*x(:, k) + B*uk + ...
                    model.pred.Ed*Dpreview + model.pred.Gd*1):['SSM_multiple_shoot_states_k',int2str(k)]];
                con = con + [ (y(:, k) == model.pred.Cd*x(:, k)+ model.pred.Dd*uk + model.pred.Fd*1):['SSM_multiple_shoot_outputs_k=',int2str(k)]];
            end
        end

        %         % comfort zone with  violation penalty - dynamic comfort zone
            
            con = con + [ (wb-s(:,k)<= y(:,k) <=wa+s(:,k)):['y_zone_k=',int2str(k)] ];
            %   input constraints
            con = con + [ (model.pred.umin <= uk <= model.pred.umax):['u_box_k=',int2str(k)] ];
        % %       slack constraints 
         con = con + [ (0*ones(model.pred.ny,1)<=s(:,k)):['nonnegative_slacks_k=',int2str(k)] ];
         
    %   -------------  OBJECTIVE FUNCTION  -------------
        %    % quadratic objective function withouth states constr.  penalisation
                obj = obj + s(:,k)'*Qsb*s(:,k) + ...         %  comfort zone penalization
                              P*(uk'*Qu*uk);                              %  quadratic penalization of ctrl action move blocking formulation
    end


     %% construction of object optimizer
     %   structure:  optimizer(constraints, objecttive, options, input_params, output_params)

    %  optimizer options
    try
        options = sdpsettings('verbose', 1, 'solver','gurobi','gurobi.TimeLimit',5);
        test = optimizer([],[],options,[],[]);
        % options = sdpsettings('verbose', 1, 'warning', 1, 'beeponproblem', 1, 'solver','cplex');
    catch
        options = sdpsettings('verbose', 1, 'solver','quadprog', 'savesolveroutput', 1);
        fprintf('Quadprog used instead \n');
    end
%   worst case optimization cpu time -  max time limit for solver options.gurobi.TimeLimit
% http://www.gurobi.com/documentation/7.5/refman/timelimit.html

     % optimizer for dynamic comfort zone
     if nd == 0  % no disturbances formulation
         mpc = optimizer(con, obj, options,  { x(:, 1),A,B, wa_prev, wb_prev, price }, {u; obj} );
     else
         mpc = optimizer(con, obj, options,  { x(:, 1),A,B, d_prev, wa_prev, wb_prev, price }, {u; obj} );
     end    

%      information about constraints size and type
     constraints_info.con = con;
     constraints_info.size = size(con);
     constraints_info.i_length = NaN(length(con),1);
     constraints_info.equality = is(con,'equality'); 
        
    for k = 1:length(con) 
        constraints_info.i_length(k) = length(double(con(k)));       
    end


end