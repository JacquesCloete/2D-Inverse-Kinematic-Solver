function rho = forward_kinematics_j2(A,B,joints)
% forward_kinematics_j2
% Computes joint 1 position (x,y) given joint angles alpha and beta, as 
% well as linkage lengths A and B

alpha = joints(1);
beta = joints(2);

x = (cosd(alpha)*cosd(beta) - sind(alpha)*sind(beta))*B + cosd(alpha)*A;

y = (sind(alpha)*cosd(beta) + cosd(alpha)*sind(beta))*B + sind(alpha)*A;

rho = [x;y];