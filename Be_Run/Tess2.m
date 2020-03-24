A = [2.9380   -0.7345    0.2500    2.9380   -0.7345    0.2500
    4.0000         0         0    4.0000         0         0
         0    1.0000         0         0    1.0000         0
    2.9380   -0.7345    0.2500    2.9380   -0.7345    0.2500
    4.0000         0         0    4.0000         0         0
         0    1.0000         0         0    1.0000         0];
B = [ 0.2500    0.2500
         0         0
         0         0
         0.2500    0.2500
         0         0
         0         0];
C = [-0.2072    0.0414    0.0726 -0.2072    0.0414    0.0726
     -0.2072    0.0414    0.0726 -0.2072    0.0414    0.0726];
E = [0.0625    0.0625    0.0625    0.0625    0.0625    0.0625    0.0625    0.0625    0.0625    0.0625    0.0625
         0         0         0         0         0         0         0         0         0         0         0
         0         0         0         0         0         0         0         0         0         0         0
    0.0625    0.0625    0.0625    0.0625    0.0625    0.0625    0.0625    0.0625    0.0625    0.0625    0.0625
         0         0         0         0         0         0         0         0         0         0         0
         0         0         0         0         0         0         0         0         0         0         0
];

nu=2;
nx=6;
ny=2;
nw=11;
N = 10;
% Construct module 

 V = sdpvar(nu,N,'full'); % in case of no uncertain disturbance U will equal V 
 W = sdpvar(nw,N,'full'); % uncertain disturbances 
 Lxx = 0.2*ones(nu,nw);
 x = sdpvar(nx, N+1, 'full');

 AB = zeros( nx , N*nu );
 AExpX0 = eye(nx) * x(:,1);
 AW = zeros( nx , N*nw); % to represent the dynamics of uncertainty W on the states X 
 Lxx = 0.2*ones(nu,nw);
 % mapping parameterisation of uncertain W on input U
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

Y = sdpvar(ny, N, 'full');
U = [];
F = [];
objective=0;
%xk = x;
for k = 1:N
    if k==1
        AB(:, (N-k)*nu+1:(N-k+1)*nu ) = B;        %  input matrix evolution
        AW(:, (N-k)*nw+1:(N-k+1)*nw) = E;     
        Y(:, k) = C*x(:, k);
        
    elseif k==2 
        AExpX0 = A* AExpX0; 
    Y(:, k) = C*( AExpX0 + AB(:, (N-k+1)*nu+1 : end ) * reshape( V(:,1:k-1) , nu * (k-1) , 1) + ...
                                   AW(:, (N-k+1)*nw+1 : end ) * reshape( W(:,1:k-1) , nw * (k-1) , 1));
                                                                  %
                                                              
                AB(:, (N-k)*nu+1:(N-k+1)*nu ) = A* AB(:, (N-k+1)*nu+1:(N-k+2)*nu );
                AW(:, (N-k)*nw+1:(N-k+1)*nw ) = A* AW(:, (N-k+1)*nw+1:(N-k+2)*nw );
    else
        AExpX0 = A* AExpX0; 
        Y(:, k) = C*( AExpX0 + AB(:, (N-k+1)*nu+1 : end ) * reshape( V(:,1:k-1) , nu * (k-1) , 1) + ...
                                   AW(:, (N-k+1)*nw+1 : end ) * reshape( W(:,1:k-1) , nw * (k-1) , 1)+ ...
                                   AB(:, (N-k+2)*nu+1 : end ) * kron((tril(ones(k-2))),Lxx)*reshape( W(:,1:k-2) , nw * (k-2) , 1));
                                                                  %
                                                              
                AB(:, (N-k)*nu+1:(N-k+1)*nu ) = A* AB(:, (N-k+1)*nu+1:(N-k+2)*nu );
                AW(:, (N-k)*nw+1:(N-k+1)*nw ) = A* AW(:, (N-k+1)*nw+1:(N-k+2)*nw );
    end
    
    F = F + [ Y(:,k) <= ones(ny,1)];
    U=(V(:,k) +  kron(([1:k-1]),Lxx) * reshape(W(:,1:k-1), nw*(k-1), 1) );
    F = F + [ -1 <= (V(:,k) +  kron(([1:k-1]),Lxx) * reshape(W(:,1:k-1), nw*(k-1), 1) ) <= 1];
    
    objective = norm(Y(:,k)-1,1) + norm(U,1)*0.01;
end

% Con


%Obj
% con uncertainity
G = [-1 <= W <= 1];
% 
%[Frobust,h] = robustify(F + G,objective,[],W);
         
xk(:,1) = [0;0;0;0;0;0];
ops = sdpsettings;
mpc=optimizer([F, G, uncertain(W)], objective, ops,  { x(:, 1) }, {V(:,1);W(:,1); objective} );

% simulation
for i = 1:25
    
    Out=mpc{xk(:,i)};
    VV(:,i)= Out{1};
    WW(:,i)= Out{2};
    IN(:,i)=(VV(:,i) +  kron(([1:i-1]),Lxx) * reshape(WW(:,1:i-1), nw*(i-1), 1) );
    IN(:,1)=Out{1};
    xk(:,i+1) = [A*xk(:,end) + B*IN(:,i) + E*(-1+2*rand(11,1))];
end










plot(C*xk)