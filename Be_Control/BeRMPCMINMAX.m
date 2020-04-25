function [mpc, constraints_info] = BeMPCdesignDraft(model, RMPCParam)
% RMPC design function using Yalmip

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
   RMPCParam.use = 0;
   RMPCParam.Condensing = 1;
   % horizons
   RMPCParam.N = 2;
   RMPCParam.Nc = 2;
   RMPCParam.Nrp = 2;
   RMPCParam.Ndp = 2;
   % weight diagonal matrices 
   RMPCParam.Qsb = 1e6*eye(model.pred.ny);
   RMPCParam.Qsa = 1e6*eye(model.pred.ny);
   RMPCParam.Qu = 1e0*eye(model.pred.nu);
end

    %% MPC parameters

    % dimensions
    nx = model.pred.nx;
    ny = model.pred.ny;
    nd = model.pred.nd;
    nu = model.pred.nu;
    nw = nd;%L2020 no of uncertain disturbance could be same as no Of know disturbance nd 
    
    % horizons   
    N = RMPCParam.N;   %  prediction horizon
    Nc = RMPCParam.Nc; %  control horizon
    Nrp = RMPCParam.Nrp; % reference preview horizon
    Ndp = RMPCParam.Ndp; % disturbacne preview horizon

    
   %%
    % variables ( inputs to Optimisation problem when calling optimiser 
    x = sdpvar(nx, N+1, 'full'); % states of the building
    d_prev = sdpvar(nd, Ndp, 'full'); % disturbances with preview
    wa_prev = sdpvar(ny, Nrp, 'full'); % above and below threshold -- dynamic comfort zone 
    wb_prev = sdpvar(ny, Nrp, 'full');
    price = sdpvar(1, Nrp, 'full'); % variable energy price profile
    y = sdpvar(ny, N, 'full');
    %  (decision variables to be found)
    V = sdpvar(nu, N,'full'); % in case of no uncertain disturbance U will equal V 
    Lxx=zeros(nu,nw);Lxx(:,1)=1;Lxx(:,7)=1;% shape your Lxx
    %LM = sdpvar(1,1,'full').*kron(tril(ones(N),-1),Lxx);% L( (k-1)*nu+1 : k*nu , 1 : (k-1)*nw) )
    LM = sdpvar(1,1,'full').*kron( (tril(ones(N),-1)-tril(ones(N),-3)),Lxx);% L( (k-1)*nu+1 : k*nu , 1 : (k-1)*nw) )
    u = sdpvar(nu, N, 'full');
    W = sdpvar(nw, N, 'full'); % uncertain disturbances 
    %sL = sdpvar(nu, N, 'full'); %  general slack
    %sH = sdpvar(nu, N, 'full'); %  general slack
    s = sdpvar(ny, N, 'full'); %  general slack
    
    
    
    % weight diagonal matrices 
    Qsb = RMPCParam.Qsb;
    Qsa = RMPCParam.Qsa;
    Qu = RMPCParam.Qu;

    %% MPC problem formulation
    %  objective function+ constraints init
    obj = 0;
    con = [];
    G=[];
    AB = zeros( nx , N*nu );
    AE = zeros( nx , N*nd );
    AG = zeros( nx , N*1);
    AExpX0 = eye(nx) * x(:,1);
    %AW = zeros( nx , N*nw); % to represent the dynamics of uncertainty W on the states X 
    %Lxx = 0.2*ones(nu,nw);  % mapping parameterisation of uncertain W on input U
                            % we should have a casual structure for U=LW+V
                            % S.T. U1=V1, U2=Lxx * W1 + V2, U3=Lxx * W1 + Lxx * W2 + V3
                            % Assuming that L10=L20=L21
                            %as SS => X= AX+BU+(G+BL)W to replace U = V+LW
                            %and still using shooting method, 
                            %V will be totally equivalent to U but BLW needs
                            %adjustment for Lxx matrix and B matrix S.T
                            %
                            %At k=3, 
                            %[AB B]  * [Lxx 0      *  W1
                                       %Lxx Lxx]      W2
                                       
                            % at K=4
                            %[A^2B AB B]      [Lxx  0   0       W1   
                            %              *   Lxx Lxx  0   *   W2
                            %                  Lxx Lxx Lxx]     W3   
                            %   
 
for k = 1:N   
   %% Input Formulation 
   
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

        if k ==1 
            u(:,k) = V(:,k);
            uk = V(:,k);
         else
            u(:,k) = V(:,k) + LM( (k-1)*nu+1 : k*nu , 1 : (k-1)*nw ) * reshape( W(:,1:k-1) , nw * (k-1) , 1);
            uk = V(:,k) + LM( (k-1)*nu+1 : k*nu , 1 : (k-1)*nw ) * reshape( W(:,1:k-1) , nw * (k-1) , 1) ;

        end
        
          %     state + output update equations
        if RMPCParam.Condensing %  Single shooting
            if k == 1
                AB(:, (N-k)*nu+1:(N-k+1)*nu ) = model.pred.Bd;        %  input matrix evolution
                AE(:, (N-k)*nd+1:(N-k+1)*nd ) =  model.pred.Ed;       %  disturbance matrix evolution
                AG(:, (N-k)*1+1:(N-k+1)*1 ) =  model.pred.Gd;         %  initial conditions matrix evolution
                AW(:, (N-k)*nw+1:(N-k+1)*nw) = ones(nx,nw);        % G matrix in paper  the impact of Uncertainty on States dynamics 
                
                y(:, k) = model.pred.Cd*x(:, k) + model.pred.Dd*uk + model.pred.Fd*1;% : ['SSM_single_shoot_k=',int2str(k)]];% L2020 using V instead of U 
            else
                AExpX0 = model.pred.Ad * AExpX0;               

                y(:, k) = model.pred.Cd*( AExpX0 + AB(:, (N-k+1)*nu+1 : end ) * reshape( u(:,1:k-1) , nu * (k-1) , 1) + ...
                                                                  AE(:, (N-k+1)*nd+1 : end ) * reshape( d_prev(:,1:k-1) , nd * (k-1) , 1) + ...
                                                                  AE(:, (N-k+1)*nw+1 : end ) * reshape( W(:,1:k-1) , nw * (k-1) , 1)+ ...
                                                                  AG(:, (N-k+1)*1+1 : end ) * ones(k-1,1) ) + ...
                                                                  model.pred.Dd*uk + model.pred.Fd*1;%:['SSM_single_shoot_k=',int2str(k)]];

                AB(:, (N-k)*nu+1:(N-k+1)*nu ) = model.pred.Ad* AB(:, (N-k+1)*nu+1:(N-k+2)*nu );
                AE(:, (N-k)*nd+1:(N-k+1)*nd ) = model.pred.Ad* AE(:, (N-k+1)*nd+1:(N-k+2)*nd );
                AG(:, (N-k)*1+1 :(N-k+1)*1 ) = model.pred.Ad* AG(:, (N-k+1)*1+1:(N-k+2)*1 );
                AW(:, (N-k)*nw+1:(N-k+1)*nw ) = model.pred.Ad* AW(:, (N-k+1)*nw+1:(N-k+2)*nw ); 
 
            end  
        else %  Multiple shooting
            if nd == 0 % no disturbances formulation
                con = con + [ (x(:, k+1) == model.pred.Ad*x(:, k) + model.pred.Bd*uk + model.pred.Gd*1):['SSM_multiple_shoot_states_k=',int2str(k)] ];
                con = con + [ (y(:, k) == model.pred.Cd*x(:, k)  + model.pred.Dd*uk + model.pred.Fd*1):['SSM_multiple_shoot_outputs_k=',int2str(k)] ];
            else
                con = con + [ (x(:, k+1) == model.pred.Ad*x(:, k) + model.pred.Bd*uk + ...
                    model.pred.Ed*Dpreview + model.pred.Gd*1):['SSM_multiple_shoot_states_k',int2str(k)]];
                con = con + [ (y(:, k) == model.pred.Cd*x(:, k)  + model.pred.Dd*uk + model.pred.Fd*1):['SSM_multiple_shoot_outputs_k=',int2str(k)] ];
            end
        end
            
        %         % comfort zone with  violation penalty - dynamic comfort zone
        %% March 2020
        %   output constraints    
         con = con + [ (wb-s(:,k)<= y(:,k) <=wa+s(:,k)):['y_zone_k=',int2str(k)] ];   
         %con = con + [ (wb-sL(:,k)<= y(:,k) <=wa+sH(:,k)):['y_zone_k=',int2str(k)] ];   

         %   input constraints
         con = con + [ (model.pred.umin <= uk  <= model.pred.umax):['u_box_k=',int2str(k)] ];% add comment here
        %   slack constraints 
         con = con + [ (0*ones(model.pred.ny,1)<=s(:,k)):['nonnegative_slacks_k=',int2str(k)] ];  
       
        %   uncertainty constraints 
         G = G + [ -10<= W(:,k) <= 10];

    %   -------------  OBJECTIVE FUNCTION  -------------
        %    % quadratic objective function withouth states constr.  penalisation
        
         obj = obj + norm(s(:,k),1) + 0.01 * norm((y(:,k)-wb),1);
         
        if k>=2                             
         obj=obj + norm(u(:,k) - u(:,k-1),1);
        end
  
                              
                             % obj = obj + s(:,k)'*Qsb*s(:,k) +V(:,k)'*Qu*V(:,k);
                                     %  quadratic penalization of ctrl action move blocking formulation
end
    



     %% construction of object optimizer
     %   structure:  optimizer(constraints, objecttive, options, input_params, output_params)

     %optimizer options
%     try
      options = sdpsettings('verbose', 1, 'solver','gurobi');%,'gurobi.TimeLimit',600);%,'gurobi.TimeLimit',50);       
%      test = optimizer([],[],options,[],[]);
%         %options = sdpsettings('verbose', 1, 'warning', 1, 'beeponproblem', 1, 'solver','cplex');
%     catch
%        options = sdpsettings;
%        fprintf('Quadprog used instead \n');
%     end
%   worst case optimization cpu time -  max time limit for solver options.gurobi.TimeLimit
% http://www.gurobi.com/documentation/7.5/refman/timelimit.html

%% March 2020 
     % optimizer for dynamic comfort zone
     if nd == 0  % no disturbances formulation
         mpc = optimizer([con, G, uncertain(W)], obj, options,  { x(:, 1), wa_prev, wb_prev, price }, {u(:,1); obj} );
     else
         mpc = optimizer([con, G, uncertain(W)], obj, options,  { x(:, 1), d_prev, wa_prev, wb_prev, price }, {u(:,1); obj} );
        
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