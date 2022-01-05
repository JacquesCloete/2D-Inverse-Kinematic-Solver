function M = find_gradient(A,B,C,joints)
% find_gradient
% Finds the gradient of the output positions x, y and theta with respect to
% the joint angles alpha, beta and gamma, given linkage lengths A, B and C,
% evaluated at the input joint angles

alpha = joints(1);
beta = joints(2);
gamma = joints(3);

dx_da = (-sind(alpha)*cosd(beta)*cosd(gamma) - cosd(alpha)*sind(beta)*cosd(gamma) + sind(alpha)*sind(beta)*sind(gamma) - cosd(alpha)*cosd(beta)*sind(gamma))*C + (-sind(alpha)*cosd(beta) - cosd(alpha)*sind(beta))*B - sind(alpha)*A;

dx_db = (-cosd(alpha)*sind(beta)*cosd(gamma) - sind(alpha)*cosd(beta)*cosd(gamma) - cosd(alpha)*cosd(beta)*sind(gamma) + sind(alpha)*sind(beta)*sind(gamma))*C + (-cosd(alpha)*sind(beta) - sind(alpha)*cosd(beta))*B;

dx_dg = (-cosd(alpha)*cosd(beta)*sind(gamma) + sind(alpha)*sind(beta)*sind(gamma) - cosd(alpha)*sind(beta)*cosd(gamma) - sind(alpha)*cosd(beta)*cosd(gamma))*C;


dy_da = (cosd(alpha)*cosd(beta)*cosd(gamma) - sind(alpha)*sind(beta)*cosd(gamma) - cosd(alpha)*sind(beta)*sind(gamma) - sind(alpha)*cosd(beta)*sind(gamma))*C + (cosd(alpha)*cosd(beta) - sind(alpha)*sind(beta))*B + cosd(alpha)*A;

dy_db = (-sind(alpha)*sind(beta)*cosd(gamma) + cosd(alpha)*cosd(beta)*cosd(gamma) - sind(alpha)*cosd(beta)*sind(gamma) - cosd(alpha)*sind(beta)*sind(gamma))*C + (-sind(alpha)*sind(beta) + cosd(alpha)*cosd(beta))*B;

dy_dg = (-sind(alpha)*cosd(beta)*sind(gamma) - cosd(alpha)*sind(beta)*sind(gamma) - sind(alpha)*sind(beta)*cosd(gamma) + cosd(alpha)*cosd(beta)*cosd(gamma))*C;

M = [dx_da dx_db dx_dg;
     dy_da dy_db dy_dg;
     1 1 1];