function ver = random_walk(number_of_tries, center)
%%function gives a random walk in 1D
%
% Input: x = random_walk(1000, 0)
%
% Output: a vector x containing the data of the
% random walk with a starting point at 0. Additionally
% the result is plotted.
%%Initialising values
%
num = number_of_tries;
yv = rand(num,1);
sum = center;
ver(1) = center;
for i = 2:num
    if yv(i)< 0.5
        sum = sum + 1;
    else
        sum = sum - 1;
    end
    ver(i) = sum;
end
%%Plot result
%
figure (1);
%hold on;
c=plot(ver, 1:num,'-rx');
set(c,'color','blue');
grid on;
title('showing the random walk less or greater than 0');
ylabel('trials');
end