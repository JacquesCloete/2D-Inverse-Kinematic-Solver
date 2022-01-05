function rho = forward_kinematics(A,B,C,joints)
% forward_kinematics
% Computes end-effector position (x,y) and orientation theta given joint
% angles alpha, beta and gamma, as well as linkage lengths A, B and C

alpha = joints(1);
beta = joints(2);
gamma = joints(3);

x = (cosd(alpha)*cosd(beta)*cosd(gamma) - sind(alpha)*sind(beta)*cosd(gamma) - cosd(alpha)*sind(beta)*sind(gamma) - sind(alpha)*cosd(beta)*sind(gamma))*C + (cosd(alpha)*cosd(beta) - sind(alpha)*sind(beta))*B + cosd(alpha)*A;

y = (sind(alpha)*cosd(beta)*cosd(gamma) + cosd(alpha)*sind(beta)*cosd(gamma) - sind(alpha)*sind(beta)*sind(gamma) + cosd(alpha)*cosd(beta)*sind(gamma))*C + (sind(alpha)*cosd(beta) + cosd(alpha)*sind(beta))*B + sind(alpha)*A;

theta = alpha + beta + gamma;

rho = [x;y;theta];