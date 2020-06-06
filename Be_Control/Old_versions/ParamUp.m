function [A,B,Objective,vx,vy] = ParamUp(model, y, u, d, x, N)


 
%% MHE parameters
        
nx = model.pred.nx;
nu = model.pred.nu;
nd = model.pred.nd;
ny = model.pred.ny;
Qe = 1e0*eye(model.pred.nx);
Re = 1e0*eye(model.pred.ny);

% variables

Aresd = sdpvar(nx,nx,'full');
Bresd = sdpvar(nx,nu,'full');
vy = sdpvar(ny, N, 'full'); %  output update error slack
vx = sdpvar(nx, N, 'full'); %  output update error slack

%% MHE problem formulation
%  objective function+ constraints init
obj = 0;
con = [];
       
% state condensing matrices initialization
for k = 1:N   
 

         con = con + [x(:,k+1) == (model.pred.Ad+Aresd)*x(:, k) + (model.pred.Bd+Bresd)*u(:, k) + model.pred.Ed*d(:, k) + model.pred.Gd*1+ vx(:, k)] ;
         con = con + [ y(:,k) == model.pred.Cd*x(:, k)+ model.pred.Dd*u(:,k)  + model.pred.Fd*1 + vy(:, k)];
        
         %    constraints   
           con = con + [  -1*ones(nx,1) <= vx(:, k) <= 1*ones(nx,1)];
           con = con + [  -1*ones(ny,1) <= vy(:, k) <= 1*ones(ny,1)];
           con = con + [  -2*ones(nx,nx) <= Aresd <= 1*ones(nx,nx)];
           con = con + [  -2*ones(nx,nu) <= Bresd <= 1*ones(nx,nu)];

       
            obj = vy(:,k)'*Re*vy(:,k)+vx(:,k)'*Qe*vx(:,k);   
%         
end
    
                
    
     %% construction of object optimizer
     %   structure:  optimizer(constraints, objecttive, options, input_params, output_params)
 
    %  optimizer options
    % options = sdpsettings('verbose', 1, 'warning', 1, 'beeponproblem', 1, 'solver','cplex');
    options = sdpsettings('verbose', 0, 'solver','gurobi');
    optimize(con,obj,options); 
    A = value(Aresd);
    B = value(Bresd);
    Objective = value(obj);
    vx = value(vx);
    vy = value(vy);

    
    
end