% Linkage Lengths

A = 0.8;

B = 0.6;

C = 0.4;


% Initial Joint Angles

alpha = 75;

beta = -45;

gamma = -40;

joints = [alpha;beta;gamma];


% Start Position

rho = forward_kinematics(A,B,C,joints);


% Plot Position

rho_j1 = forward_kinematics_j1(A,joints(1));

rho_j2 = forward_kinematics_j2(A,B,joints(1:2));

plot([0 rho_j1(1) rho_j2(1) rho(1)],[0 rho_j1(2) rho_j2(2) rho(2)],'k','LineWidth',3)
hold on
plot([rho_j1(1) rho_j2(1)],[rho_j1(2) rho_j2(2)],'ko','LineWidth',3)
plot(0,0,'k^','LineWidth',3)
gripper(rho)


% End Goal

rho_final = [(1+3*rand(1))/4;(1+3*rand(1))/4;-30];


% Plot End Goal

plot(rho_final(1),rho_final(2),'rx')
xlim([-0.5, 1.5]);
ylim([-0.5, 1.5]);
grid on
axis square

pause(1e-5);
disp('3')
pause(1)
disp('2')
pause(1)
disp('1')
pause(1)
disp('Iteration Process Started')


% Set Iteration Limit

it_limit = 5000;


% Set Fractional Increment Amount

eta = [0.2; 0.2; 0.004];


% Initialise Iteration Counter
it = 0;

% begin Process
while ((abs(rho(1) - rho_final(1)) > 0.001) || (abs(rho(2) - rho_final(2)) > 0.001) || (abs(rho(3) - rho_final(3)) > 0.02)) && it < it_limit 
    
    % Increment Iteration Counter
    
    it = it + 1;
    
    
    % Set Target Position for This Iteration
    
    rho_target = rho + exp(it/200)*eta.*(rho_final - rho);   % Simple linear interpolation scheme (could be interesting to experiment with other schemes)
    
    
    % Find the Gradient at the Current Position
    
    M = find_gradient(A,B,C,joints);
    
    
    % Solve for the Required Changes in Joint Angles
    
    d_joints = M\(rho_target - rho);
    
    
    % Update the Joint Angles
    
    joints = joints + d_joints;
    
    
    % Compute the New End-Effector Position
    
    rho = forward_kinematics(A,B,C,joints);
    
    
    % Plot Position
    
    rho_j1 = forward_kinematics_j1(A,joints(1));
    
    rho_j2 = forward_kinematics_j2(A,B,joints(1:2));
    
    
    hold off
    plot([0 rho_j1(1) rho_j2(1) rho(1)],[0 rho_j1(2) rho_j2(2) rho(2)],'k','LineWidth',3)
    hold on
    plot([rho_j1(1) rho_j2(1)],[rho_j1(2) rho_j2(2)],'ko','LineWidth',3)
    plot(0,0,'k^','LineWidth',3)
    gripper(rho)
    
    
    % Plot End Goal
    
    plot(rho_final(1),rho_final(2),'rx')
    xlim([-0.5, 1.5]);
    ylim([-0.5, 1.5]);
    grid on
    axis square
    
    pause(1e-5);
    
end

fprintf('Trajectory Solved in %d Iterations\n',it)
