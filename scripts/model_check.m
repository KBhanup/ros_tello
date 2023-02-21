
clc
clear all
load("models.mat")

disp("Controllability...")

N=size(ss.A,1);
M=[ss.B,zeros(N,N-1)];
for i=1:N-1
   M(:,i+1) = ss.A*M(:,i); 
end
if(rank(M)==N)
    disp("Controllable")
else
    disp("Not Controllable")
end
