function [x,y] = chicane(t)

x = nan(size(t));
y = nan(size(t));

tfin = 5;
w = 2*pi * 2/tfin;
A = tfin/16;

t1 = tfin/4;
t2 = tfin/2;
t3 = 3*tfin/4;

% First line
x(t<=t1) = 0;
y(t<=t1) = t(t<=t1)*A*w;

% First circle
dx1 = 0;
dy1 = A*w*t1;

x((t>t1) & (t<=t2)) = dx1 + A * (1-cos(w*(t((t>t1) & (t<=t2))-t1)));
y((t>t1) & (t<=t2)) = dy1 + A * sin(w*(t((t>t1) & (t<=t2))-t1));

% Second circle
dx2 = dx1 + A * (1-cos(w*(t2-t1)));
dy2 = dy1 + A * sin(w*(t2-t1));
    
x((t>t2) & (t<=t3)) = dx2 + A * (1-cos(w*(t((t>t2) & (t<=t3))-t2)));
y((t>t2) & (t<=t3)) = dy2 - A * sin(w*(t((t>t2) & (t<=t3))-t2));

% Final line
dx3 = dx2 + A * (1-cos(w*(t3-t2)));
dy3 = dy2 - A * sin(w*(t3-t2));

x(t>t3) = dx3;
y(t>t3) = dy3 + A*w*(t(t>t3)-t3);


% if t <= t1
%     % First line
%     x = 0;
%     y = t*A*w;
% elseif t <= t2
%     x = 0;
%     y = t1*A*w;
%     % First circle
%     x = x + A * (1-cos(w*(t-t1)));
%     y = y + A * sin(w*(t-t1));
% elseif t <= t3
%     x = 0;
%     y = t1*A*w;
%     % First circle
%     x = x + A * (1-cos(w*(t2-t1)));
%     y = y + A * sin(w*(t2-t1));
%     
%     % Second circle
%     x = x + A * (1-cos(w*(t-t2)));
%     y = y - A * sin(w*(t-t2));
% else
%     x = 0;
%     y = t1*A*w;
%     % First circle
%     x = x + A * (1-cos(w*(t2-t1)));
%     y = y + A * sin(w*(t2-t1));
%     
%     % Second circle
%     x = x + A * (1-cos(w*(t3-t2)));
%     y = y - A * sin(w*(t3-t2));
%     % Final line
%     x = x;
%     y = y + (t-t3)*A*w; %+ (tfin-t3+1);
% end
