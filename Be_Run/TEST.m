    % dimensions
    nx = 20;
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
    r = sdpvar(ny, 1, 'full'); %refernce
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

    
    Qy = 1e-5*eye(nu);%-3 *e0
    Qw = 1e6*eye(nx);%C{3,1}'* C{3,1}*1e1 % -2 % 1e-2*eye(nu)
    Qsb= 1e0*eye(ny);
    
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
%    %con = con + [(0<=s(i,1)):['nonnegative_slacks_k=',int2str(i)] ]; 
%      con = con + [(290<=( C{1,1}*x )<=300):['Output1']];
%      con = con + [(290<=( C{2,1}*x )<=300):['Output2']];
%     con = con + [(290<=( C{3,1}*x )<=300):['Output3']];
%                            
% Constraints
con = con + Lmi_Lyap + Lmi_rie + Lmi_convix + Lmi_u_max + Lmi_output_max;
   
    %   -------------  OBJECTIVE FUNCTION  -------------
                            obj = obj  +Gamma; 
                            
                            
                            obj = obj +(r - C{1,1}*x )' * Qsb * (r - C{1,1}*x);
                            obj = obj +(r - C{2,1}*x )' * Qsb * (r - C{2,1}*x);
                            obj = obj +(r - C{3,1}*x )' * Qsb * (r - C{3,1}*x);
                            
    end

        options = sdpsettings('verbose', 1, 'solver','sedumi');%,'gurobi.TimeLimit',50);
        
       % sol = optimize(con,obj, options);
%         UsedInObjective = recover(depends(obj));
%         optimize([con, -1000000 <= UsedInObjective <= 1000000] ,obj)
%         value(UsedInObjective)
%         
        solver = optimizer(con,obj, options,{x(:,1),r},{Y(:,1:nx),W(:,1:nx)});
        mpc=solver;
        
        
        

        
        
         output=solver{{ones(20,1),[295;295;295;295;295;295]}};
         yy=double(output{1,1});
         ww=output{1,2};
%         gg=output{1,3};
 %        SSS=output{1,4};
         F=yy*ww^-1
  %       value (slack1)
         
         
         
         vv= [       -0.0047
   -0.0121
   -0.0047
   -0.0066
   -0.0027
    0.0035
    0.0014
   -0.0013
    0.0013
    0.0003
    0.0014
    0.0089
   -0.0027
    0.0021
   -0.0018
   -0.0035
   -0.0005
   -0.0020
   -0.0011
    0.0014];

DD=[    0.0245
    0.1304
    0.5050
   -0.8184
   -1.0655
    0.3822
    0.3337
    0.3502
   -0.7443
   -2.9816];
% 
%    output1=solver(DD);

%          yy1=output1{1,1};
%          ww1=output1{1,2};
%          gg1=output{1,3};
%          SSS1=output{1,4};
%          
%          F1=yy1*ww1^-1
%          
%         

R=refs.R';
model2=model;
    M = inv([model2.pred.Ad - eye(model2.pred.nx), model2.pred.Bd; model2.pred.Cd, model2.pred.Dd]);
                
                
  

RR=R(:,34:96*7+33);

D = dist.d(1:96*7,:)';
for k= 1 : 96*7
    d0 = D(:,k);
    SS(:,k)= M *[-(model2.pred.Ed*d0+model2.pred.Gd); [R(:,k) - model2.pred.Fd]];
end
xss = SS(1:model2.pred.nx,:);
uss = SS(model2.pred.nx+1:end,:);
for k= 1 : 96*7
    d0 = D(:,k);
    SSS(:,k)= M *[-(model2.pred.Ed*d0+model2.pred.Gd); [[297;297;297;297;297;297] - model2.pred.Fd]];
end

xsss = SSS(1:model2.pred.nx,:);
usss = SSS(model2.pred.nx+1:end,:);



figure
plot(t,uss,'r')
hold on
plot(t,usss,'b-.')

figure 
plot(t,R(:,1:96*7),'r')




283.98



s=rng;
W=mean(dist.d(:,41)-normrnd(mean(dist.d(:,41)),var(dist.d(:,41)),1,600);




