function gripper(rho)
% gripper
% Plots the gripper, given end-effector position (x,y) and orientation
% theta

x = rho(1);
y = rho(2);
theta = rho(3);

gripper_coords = [0.1 0.04 0 0.04 0.1;
           0.025 0.05 0 -0.05 -0.025]*1.5;
       
gripper_coords = [cosd(theta) -sind(theta); sind(theta) cosd(theta)]*gripper_coords;

gripper_coords(1,:) = gripper_coords(1,:) + x;
gripper_coords(2,:) = gripper_coords(2,:) + y;

plot(gripper_coords(1,:),gripper_coords(2,:),'k','LineWidth',3)