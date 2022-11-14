%% INPUT
y = @(t) [cos(0.5.*t)+cos(0.2.*t); sin(1.1.*t)+sin(t)];     % trajectory
%y = @(t) [t; t.*t];     % trajectory
t_f = 10;  % final instant
N = 200;    % number of sampling instants

%% Processing
p_size = size(y(0),1);

[tK, refs, minCost] = optQuanta(y, N, t_f);

%% Plotting
delta_t = 0.01;
t=[0:delta_t:t_f];
y(t);

% trajectory in 2D
figure(1);
plot(ans(1,:),ans(2,:),'b-','LineWidth',2);
hold on;
y_at_change = y(tK);
plot(y_at_change(1,:),y_at_change(2,:),'k+');
plot(refs(1,:),refs(2,:),'r.',"MarkerSize",15)
title('Trajectory');
xlabel('X coordinate');
ylabel('Y coordinate');
hold off;
axis equal;

% trajectory over time
figure(2);
for i=1:p_size
	subplot(p_size,1,i);
	plot(t,ans(i,:),'b-','LineWidth',2);
	hold on;
	plot(tK,y_at_change(i,:),'k+');
	for k=1:N
		plot([tK(k) tK(k+1)], [refs(i,k) refs(i,k)], 'r-','LineWidth',2);
	end
	ylabel(strcat("coordinate ",string(i)));
	xlabel('time');
	hold off;
end

% density
figure(3);
for k=1:N
	plot([tK(k) tK(k+1)], [1 1]./(N*(tK(k+1)-tK(k))), 'r-','LineWidth',2);
	hold on;
end
hold off;
