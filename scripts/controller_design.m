clc
clear all

load('models.mat')

XQ=1;
YQ=1;
ZQ=1;
WQ=1;

XR=1;
YR=1;
ZR=1;
WR=1;

Q=diag([XQ,YQ,ZQ,WQ]);
R=diag([XR,YR,ZR,WR]);

A=eye(4)
B=eye(4)*.05

controller = dlqr(A,B,Q,R)

writematrix(controller,"controller.csv")
save('controller.mat','controller')