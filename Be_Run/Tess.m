A = [2.938 -0.7345 0.25;4 0 0;0 1 0;];
B = [0.25;0;0];
C = [-0.2072 0.04141 0.07256];
E = [0.0625;0;0];

% Construct module 

N = 10;
%U = sdpvar(N,1);
W = sdpvar(N,1);
x = sdpvar(3,1);

V = sdpvar(N,1);
L = sdpvar(N,N,'full').*(tril(ones(N))-eye(N));

U = L*W + V;

Y = [];
xk = x;
for k = 1:N
 xk = A*xk + B*U(k)+E*W(k);
 Y = [Y;C*xk];
end
% Con

F = [Y <= 1, -1 <= U <= 1];
%Obj
objective = norm(Y-1,1) + norm(U,1)*0.01;
% con uncertainity
G = [-1 <= W <= 1];
% 
[Frobust,h] = robustify(F + G,objective,[],W);

 xk = [0;0;0];
ops = sdpsettings;
% simulation
for i = 1:25
    optimize([Frobust, x == xk(:,end)],h,ops);
    xk = [xk A*xk(:,end) + B*value(U(1)) + E*(-1+2*rand(1))];
end
plot(C*xk)