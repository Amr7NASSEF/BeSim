for i= 1:44
    mu(:,i)=mean(dist.d(1+k:32+k,i));
    sig(:,i)=var(dist.d(1+k:32+k,i));
    W(1,i)=normrnd(mu(1,i),0);
end