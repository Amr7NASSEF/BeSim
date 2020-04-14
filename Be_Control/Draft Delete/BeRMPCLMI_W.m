function [mpc, constraints_info] = BeRMPCLMI_W(model1,model2,model3 )
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
   
    Qy = eye(nu)*10^-3;
    Qw = C{1,1}'*C{1,1}*10^5;
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

        options = sdpsettings('verbose', 1, 'solver','sedumi');       
        mpc = optimizer(con,obj, options,{x,umax,ymax},{Y,W,Gamma});
        
             constraints_info.con = con;
     constraints_info.size = size(con);
     constraints_info.i_length = NaN(length(con),1);
     constraints_info.equality = is(con,'equality'); 
        
    for k = 1:length(con) 
        constraints_info.i_length(k) = length(double(con(k)));       
    end
end
