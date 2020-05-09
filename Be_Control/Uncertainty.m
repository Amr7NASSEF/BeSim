function [W] = Uncertainty(Dis,NSIM,Ts)% RMPCLMIParam
D = Dis';
Nsim=NSIM;
%W = mean(D(41,1:30*Nsim)) - normrnd(mean(D(41,1:30*Nsim)),0.5*var(D(41,1:30*Nsim)),1,30*Nsim);
W = mean(D(41,:)) - normrnd(mean(D(41,:)),0.5*var(D(41,:)),1,length(D(41,:)));
for i=1:length(D)-1
    if W(i)>20
        W(i)=20;
    elseif W(i)<-20
            W(i)=-20;
    else
    end
    if abs(W(i+1) - W(i))>0.5
        if (W(i+1) - W(i))> 0
            W(i+1)= W(i)+0.5;
        else
            W(i+1)= W(i)-0.5;
        end
    end
end
tt=(1:Nsim)*Ts/3600/24;
figure;
plot(tt,W(:,1:Nsim)); 
title('Ambient Temperature Uncertainties');
ylabel('Temperature')%[^{\circ}C]'
xlabel('time [days]')%
end