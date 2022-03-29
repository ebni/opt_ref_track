load("scenario1.mat"); es1 = es; ts1 = times(1:end-1);
load("scenario2.mat"); es2 = es; ts2 = times(1:end-1);
load("scenario3.mat"); es3 = es; ts3 = times(1:end-1);
esCell = {es1, es2, es3}; tsCell = {ts1, ts2, ts3};
labels = ["10","4","2"];

close all;
cmap = gray(4);
figure(1); hold on; grid on; box on; set(gca,'FontSize',20,'FontName','Times New Roman');
xlabel("$t$","Interpreter","latex"); ylabel("$e(t) = |\tilde{y}(t)-y(t)|$","Interpreter","latex");
for i = 1:3
    plot(tsCell{i},esCell{i},'Color',cmap(i,:),"LineWidth",3);
end
legend(["\tau=0.1","\tau=0.25","\tau=0.5"],"Location","northwest");