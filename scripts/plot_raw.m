clc
clearvars

traj = readmatrix("traj.csv");
ctrl = readmatrix("ctrl.csv");

if(traj(1,1)<ctrl(1,1))
    base = traj(1,1);
else
    base = ctrl(1,1);
end
traj(:,1) = traj(:,1) - base;
ctrl(:,1) = ctrl(:,1) - base;

figure(1)
clf
for i=1:4
    subplot(4,1,i)
    plot(traj(:,1),traj(:,i+1));
    hold on;
    plot(ctrl(:,1),ctrl(:,i+1));
end