clc
clear all
traj_orig = readmatrix("traj.csv");
ctrl = readmatrix("ctrl.csv");
if(traj_orig(1,1)<ctrl(1,1))
    base = traj_orig(1,1);
else
    base = ctrl(1,1);
end
traj_orig(:,1)=traj_orig(:,1)-base;
ctrl(:,1) = ctrl(:,1)-base;

%Load
traj_xyzw = tum_to_xyzw(traj_orig);

%Filter
x_0=[traj_xyzw(1,2:5),0,0,0,0].';
P_0=eye(8,8);


pNoise = 1e-2;
wNoise = 1e-2;
pdNoise = 1e1;
wdNoise = 1e1;
W=diag([pNoise,pNoise,pNoise,wNoise,pdNoise,pdNoise,pdNoise,wdNoise]);

puNoise = 1e1;
wuNoise = 1e1;
V=diag([puNoise,puNoise,puNoise,wuNoise]);

N=size(traj_xyzw,1);

traj_filt = zeros(N-1,9);
filt = TelloKF(x_0,P_0);
for i=1:(N-1)
    filt.predict(traj_xyzw(i+1,1)-traj_xyzw(i,1),W);
    filt.update(traj_xyzw(i+1,2:5).',V);
    traj_filt(i,:)=[traj_xyzw(i+1,1),filt.x.'];
end
writematrix(ctrl,"ctrl_aligned.csv")
writematrix(traj_filt,"traj_filtered.csv")

% Plot unfiltered vs filtered

figure(1)
clf
plot_filter = true;
plot_orig = true;

for i=1:4
    subplot(4,1,i)
    if(plot_filter)
        plot(traj_filt(:,1),traj_filt(:,i+1))
        hold on;
    end
    if(plot_orig)
        plot(traj_xyzw(:,1),traj_xyzw(:,i+1),":")
        hold on;
    end
    if(plot_orig && plot_filter)
        legend(["Filter" "Original"])
    end
    hold off;
end

figure(2)
clf
plot_filter = true;
plot_orig = true;

for i=1:4
    subplot(4,1,i)
    if(plot_filter)
        plot(traj_filt(:,1),traj_filt(:,i+5))
        hold on;
    end
    if(plot_orig)
        dv = traj_xyzw(2:end,i+1)-traj_xyzw(1:end-1,i+1);
        dt = traj_xyzw(2:end,1)-traj_xyzw(1:end-1,1);
        dvdt = dv./dt;
        plot(traj_xyzw(2:end,1),dvdt,":");
        hold on;
    end
    if(plot_orig && plot_filter)
        legend(["Filter" "Original"])
    end
end
