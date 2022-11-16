clear; clc;

tfin = 5;
t = 0:.01:tfin;
n = length(t);

x = nan(n,1);
y = nan(n,1);

for i=1:n
    [x(i),y(i)] = chicane(t(i));
end
% 
% % First line
% t1 = round(tfin/4);
% x(1:t1) = 0;
% y(1:t1) = 0:(t1-1);
% 
% % First circle
% t2 = round(2*tfin/4);
% x(t1+1:t2) = x(t1) +200*(1-cos(2*pi*4/tfin*(0:.5:t1/2-0.5)));
% y(t1+1:t2) = y(t1) + 200*sin(2*pi*4/tfin*(0:.5:t1/2-0.5));
% 
% % Second circle
% t3 = round(3*tfin/4);
% x(t2+1:t3) = x(t2) + 200*(1-cos(2*pi*4/tfin*(0:.5:t1/2-0.5)));
% y(t2+1:t3) = y(t2) - 200*sin(2*pi*4/tfin*(0:.5:t1/2-0.5));
% 
% % Final line
% x(t3+1:tfin) = x(t3);
% y(t3+1:tfin) = linspace(y(t3),tfin,length(t3+1:tfin)); %+ (tfin-t3+1);
% 

figure(1);
plot(x,y)

figure(2);
%t = 0:tfin-1;
subplot(211);
plot(t,x);
subplot(212);
plot(t,y);