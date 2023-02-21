clc
clear all
ctrl = readmatrix("ctrl_lerp.csv");
traj_filt = readmatrix("traj_lerp_filt.csv");

traj=traj_filt;
traj(1,2:5)=[0,0,0,0];
for i=2:size(traj_filt,1)
    dxy=traj_filt(i,2:3)-traj_filt(i-1,2:3);
    
    th = traj_filt(i-1,5);
    rot= [cos(th),-sin(th);sin(th),cos(th)];
    
    rot_dxy=(rot*(dxy.')).';
    
    traj(i,2:3)=traj(i-1,2:3)+rot_dxy;
    traj(i,4) = traj_filt(i,4)-traj_filt(i-1,4)+traj(i-1,4);
    traj(i,5) = traj_filt(i,5)-traj_filt(i-1,5)+traj(i-1,5);
    
    while(traj(i,5)-traj(i-1,5)>pi)
        traj(i,5)=traj(i,5)-2*pi;
    end
    while(traj(i,5)-traj(i-1,5)<-pi)
        traj(i,5)=traj(i,5)+2*pi;
    end
end

%pos,vel,ctrl
figure(1)
clf
for i=1:4
    subplot(4,1,i)
    plot(traj(:,1),traj(:,i+1),ctrl(:,1),ctrl(:,i+1),"--")
    legend("Pos","Ctrl")
end


Ts=.05;

disp("Estimating x model")
xdata = iddata(traj(:,[2 6]),ctrl(:,2),Ts);
xdatad = detrend(xdata);
xss = ssest(xdata,2);
fprintf("SS Focus: %f\n",xss.Report.Fit.MSE)

disp("Estimating y model")
ydata = iddata(traj(:,[3 7]),ctrl(:,3),Ts);
ydatad = detrend(ydata);
yss = ssest(ydata,2);
fprintf("SS Focus: %f\n",yss.Report.Fit.MSE)

disp("Estimating z model")
zdata = iddata(traj(:,[4 8]),ctrl(:,4),Ts);
zdatad = detrend(zdata);
zss = ssest(zdata,2);
fprintf("SS Focus: %f\n",zss.Report.Fit.MSE)

disp("Estimating w model")
wdata = iddata(traj(:,[5 9]),ctrl(:,5),Ts);
wdatad = detrend(wdata);
wss = ssest(wdata,2);
fprintf("SS Focus: %f\n",wss.Report.Fit.MSE)

A=blkdiag(xss.A,yss.A,zss.A,wss.A)
B=blkdiag(xss.B,yss.B,zss.B,wss.B)
save('models.mat','A','B')