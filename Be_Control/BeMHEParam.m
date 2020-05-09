function mhe = BeMHEParam(model, MHEParams)

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
   MHEParams.N = 1; %  estimation horizon  
   % weights - covariances
   Qe = 1e6;                                     % process noise covariance magnitude
   Re = 1e0;                                     % measurement noise covariance magnitude
   MHEParams.Qe = Qe*eye(model.pred.nx);                            % process noise covariance 
   MHEParams.Re = Re*eye(model.pred.ny);                             % measurement noise covariance 
   MHEParams.P = model.pred.Bd*Qe*model.pred.Bd';                   % error covariance of the arrival cost  
   MHEParams.Condensing = 0;
end

 
%% MHE parameters
        
nx = model.pred.nx;
nu = model.pred.nu;
nd = model.pred.nd;
ny = model.pred.ny;
N =  MHEParams.N;
Qe = MHEParams.Qe;
Re = MHEParams.Re;

% variables
u = sdpvar(nu, N, 'full'); % ctrl actions 
d = sdpvar(nd, N, 'full'); % disturbances  
y = sdpvar(ny, N, 'full'); % outputs 
Aresd = sdpvar(nx,nx,'full');
Bresd = sdpvar(nx,nu,'full');
x = sdpvar(nx, N+1, 'full'); % states of the building
vy = sdpvar(ny, N, 'full'); %  output update error slack
vx = sdpvar(nx, N, 'full'); %  output update error slack

%% MHE problem formulation
%  objective function+ constraints init
obj = 0;
con = [];
       
% state condensing matrices initialization
for k = 1:N   
 
        
        %   -------------  Constraints  -------------
        
       %     state + output update equations
%         if MHEParams.Condensing 
%             if k == 1
%                 AB(:, (N-k)*nu+1:(N-k+1)*nu ) = model.pred.Bd;        %  input matrix evolution
%                 AE(:, (N-k)*nd+1:(N-k+1)*nd ) =  model.pred.Ed;       %  disturbance matrix evolution
%                 AG(:, (N-k)*1+1:(N-k+1)*1 ) =  model.pred.Gd;         %  initial conditions matrix evolution
%                 con = con + [ y(:, k) == model.pred.Cd*x(:, k)  + model.pred.Dd*u(:, k) + model.pred.Fd*1  + v(:, k)];
%             else
%                 AExpX0 = model.pred.Ad * AExpX0;
%                 con = con + [ y(:, k) == model.pred.Cd*( AExpX0 + AB(:, (N-k+1)*nu+1 : end ) * reshape( u(:,1:k-1) , nu * (k-1) , 1) + ...
%                                                                  AE(:, (N-k+1)*nd+1 : end ) * reshape( d(:,1:k-1) , nd * (k-1) , 1) + ...
%                                                                  AG(:, (N-k+1)*1+1 : end ) * ones(k-1,1) + Aresd*AExpX0 +Bresd *u(:,k) + v(:,k) ) + ...
%                                                                  model.pred.Dd*u(:, k)  + model.pred.Fd*1  ];
% 
%                 AB(:, (N-k)*nu+1:(N-k+1)*nu ) = model.pred.Ad* AB(:, (N-k+1)*nu+1:(N-k+2)*nu );
%                 AE(:, (N-k)*nd+1:(N-k+1)*nd ) = model.pred.Ad* AE(:, (N-k+1)*nd+1:(N-k+2)*nd );
%                 AG(:, (N-k)*1+1:(N-k+1)*1 )   = model.pred.Ad* AG(:, (N-k+1)*1+1:(N-k+2)*1 );
%             end    
        %else                
          con = con + [ x(:,k+1) == Aresd*x(:, k) + Bresd*u(:, k) + model.pred.Ed*d(:, k) + model.pred.Gd*1 + vx(:, k)];
          con = con + [y(:,k) == model.pred.Cd*x(:, k)+ model.pred.Dd*u(:,k)  + model.pred.Fd*1 + vy(:, k)];
         %   y(:,k) = model.pred.Cd*x(:, k)+ model.pred.Dd*u(:,k)  + model.pred.Fd*1+ vy(:, k);
        %end
         %con = con + [x(:,k+1) == (model.pred.Ad+Aresd)*x(:, k) + (model.pred.Bd+Bresd)*u(:, k) + model.pred.Ed*d(:, k) + model.pred.Gd*1+ vx(:, k)] ;
         %y(:,k) = model.pred.Cd*x(:, k)+ model.pred.Dd*u(:,k)  + model.pred.Fd*1+ vy(:, k);
         %con = con + [y(:,k) == model.pred.Cd*x(:, k)+ model.pred.Dd*u(:,k)  + model.pred.Fd*1 + vy(:, k)];
        
        
        % output estimation error penalization constraints        
        
        
         %    constraints   
           con = con + [  -1 <= vx(:, k) <= 1];
           con = con + [  -1 <= vy(:, k) <= 1];
         
       
           obj = 0.01*norm(y(:,k),293.15); %vy(:,k)'*Re*vy(:,k) + vx(:,k)'*Qe*vx(:,k); %0.01*norm(y(:,k),293.15);
%         
end

%                     for i = 1:nx
%                         for j=1:nx
%                             con = con+[ -2<=Aresd(i,j) <= 1];
%                         end
%                     end
                    con = con + [  -2 <= Aresd <= 1];
                    con = con + [  -2 <= Bresd <= 1];

    
                 %  arrival cost term to the first element in the horizon (x_{k-N+1})
    
     %% construction of object optimizer
     %   structure:  optimizer(constraints, objecttive, options, input_params, output_params)
 
    %  optimizer options
    % options = sdpsettings('verbose', 1, 'warning', 1, 'beeponproblem', 1, 'solver','cplex');
    options = sdpsettings('verbose', 1, 'solver','gurobi');
    mhe = optimizer(con, obj, options,  {y, u, d, x}, {Aresd; Bresd; obj;vx;vy } ); 

    
    
end