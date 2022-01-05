function rho = forward_kinematics_j1(A,alpha)
% forward_kinematics_j1
% Computes joint 1 position (x,y) given joint angle alpha as well as 
% linkage length A

x = cosd(alpha)*A;

y = sind(alpha)*A;

rho = [x;y];
