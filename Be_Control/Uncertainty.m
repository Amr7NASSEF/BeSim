function [W] = Uncertainty(Dis,NSIM,Ts)% RMPCLMIParam
D = Dis;
Nsim=NSIM;
W = mean(D(41,1:Nsim)) - normrnd(mean(D(41,1:Nsim)),0.5*var(D(41,1:Nsim)),1,Nsim);
for i=1:length(W)-1
    if W(i)>100
        W(i)=100;
    elseif W(i)<-100
            W(i)=-100;
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
plot(tt,W); 
title('Ambient Temperature Uncertainties');
ylabel('Temperature')%[^{\circ}C]'
xlabel('time [days]')%
end