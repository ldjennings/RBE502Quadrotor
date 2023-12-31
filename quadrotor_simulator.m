%% Initializations
clc; clear; close all;
clear functions;

g = 9.81;   % The gravitational acceleration [m/s^2]
l =  0.2;   % Distance from the center of mass to each rotor [m]
m =  0.5;   % Total mass of the quadrotor [kg]
I = [1.24, 1.24, 2.48]; % Mass moment of inertia [kg m^2]
mu = 3.0;   % Maximum thrust of each rotor [N]
sigma = 0.01; % The proportionality constant relating thrust to torque [m]


pd = [g l m I mu sigma];


% Initial conditions
%     x y z phi theta psi Vx Vy Vz W1 W2 W3
z0 = [0 0  0  0    0  0  0  0  0  0  0  0]';


r = [0; 0; 0;];
n = [0; 0; 0;];
u = ones(4,1) * (m*g)/4;


%% Solving the initial-value problem
t2 = 18;
dt = 1/200;
t = linspace(0, t2, t2/dt);

[t,z] = ode45(@(t,z) quadrotor(t, z, u, pd, r, n), t, z0);


%% Plotting the results

fsize = 18;

for i=1:4
    ax(i) = subplot(2,2,i,'NextPlot','Add','Box','on','XGrid','on','YGrid','on',...
                'Xlim',[t(1), t(end)],...
                'TickLabelInterpreter','LaTeX','FontSize',fsize);
    xlabel('t','Interpreter','LaTeX','FontSize',fsize);        
end


plot(ax(1), t,z(:,1:3), 'LineWidth', 1.5);
legend(ax(1), {'$x_1$', '$x_2$', '$x_3$'},... 
    'Interpreter', 'LaTeX', 'FontSize', fsize);
title(ax(1), '${\bf x}$','Interpreter','LaTeX','FontSize',fsize);
xlabel(ax(1), 't','Interpreter','LaTeX','FontSize',fsize);
grid on;

plot(ax(3), t, z(:,4:6), 'LineWidth', 1.5);
legend(ax(3), {'$\phi$', '$\theta$', '$\psi$'},...
    'Interpreter', 'LaTeX', 'FontSize', fsize);
title(ax(3), '$\mathbf\alpha$','Interpreter','LaTeX','FontSize',fsize);
grid on;

plot(ax(2), t, z(:,7:9), 'LineWidth', 1.5);
legend(ax(2), {'$\dot{x}_1$', '$\dot{x}_2$', '$\dot{x}_3$'},...
    'Interpreter', 'LaTeX', 'FontSize', fsize);
title(ax(2), '$\dot{\bf x}$','Interpreter','LaTeX','FontSize',fsize);
grid on;

plot(ax(4), t, z(:,10:12), 'LineWidth', 1.5);
legend(ax(4), {'$\omega_1$', '$\omega_2$', '$\omega_3$'},...
    'Interpreter', 'LaTeX', 'FontSize', fsize);
title(ax(4), '$\mathbf\omega$','Interpreter','LaTeX','FontSize',fsize);
grid on;



%% Animation

% Preallocate arrays 
xd = zeros(size(t));
yd = zeros(size(t));
zd = zeros(size(t));

iscaptured = false;
for k = 1:length(t)

    if iscaptured
        xd(k) = z(k,1);
        yd(k) = z(k,2);
        zd(k) = z(k,3);
    else
        pd = UAV_Trajectory(t(k));
        xd(k) = pd(1);
        yd(k) = pd(2);
        zd(k) = pd(3);
        p = z(k,1:3)';
        iscaptured = norm(p-pd,2) <= .1;
    end
end

animation_fig = figure;


airspace_box_length = 4;

animation_axes = axes('Parent', animation_fig,...
    'NextPlot','add','DataAspectRatio',[1 1 1],...
    'Xlim',airspace_box_length*[-0.5 0.5],...
    'Ylim',airspace_box_length*[-0.5 0.5],...
    'Zlim',airspace_box_length*[0 1],...
    'box','on','Xgrid','on','Ygrid','on','Zgrid','on',...
    'TickLabelInterpreter','LaTeX','FontSize',14);

plot3(xd, yd, zd, 'LineStyle', '-', 'Color', "#0072BD", 'LineWidth', 2, 'Parent',animation_axes);
plot3(z(:,1), z(:,2), z(:,3), 'LineStyle', '-.', 'Color', "#77AC30", 'LineWidth', 2, 'Parent',animation_axes);
xlim([-5, 5]); % Set x-axis limits
ylim([-5, 5]); % Set y-axis limits
zlim([0, 10]);     % Set z-axis limits

view(animation_axes, 3);

N = 10;
Q = linspace(0,2*pi,N)';
circle = 0.3*l*[cos(Q) sin(Q) zeros(N,1)];
loc = l*[1 0 0; 0 1 0; -1 0 0; 0 -1 0];


silhouette = plot3(0,0,0, '--', 'Color', 0.5*[1 1 1], 'LineWidth', 1 ,...
    'Parent', animation_axes);
body = plot3(0,0,0, 'Color',lines(1), 'LineWidth', 2,...
        'Parent', animation_axes);
for i=1:4
    rotor(i) = plot3(0,0,0, 'Color', lines(1), 'LineWidth', 2,...
        'Parent', animation_axes);
end

UAV = plot3(xd(1), yd(1), zd(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'Parent', animation_axes);


tic;
for k=1:length(t)

    
    R = [ cos(z(k,5))*cos(z(k,6)), sin(z(k,4))*sin(z(k,5))*cos(z(k,6)) - cos(z(k,4))*sin(z(k,6)), sin(z(k,4))*sin(z(k,6)) + cos(z(k,4))*sin(z(k,5))*cos(z(k,6));
          cos(z(k,5))*sin(z(k,6)), cos(z(k,4))*cos(z(k,6)) + sin(z(k,4))*sin(z(k,5))*sin(z(k,6)), cos(z(k,4))*sin(z(k,5))*sin(z(k,6)) - sin(z(k,4))*cos(z(k,6));
                     -sin(z(k,5)),                                 sin(z(k,4))*cos(z(k,5)),                                 cos(z(k,4))*cos(z(k,5))];
    for i=1:4
        ctr(i,:) = z(k,1:3) + loc(i,:)*R';
        pose = ones(N,1)*z(k,1:3) + (ones(N,1)*loc(i,:) + circle)*R';
        set(rotor(i), 'XData', pose(:,1), 'YData', pose(:,2),  'ZData', pose(:,3) );
         
    end
    set(silhouette,'XData', [0, z(k,1), z(k,1), z(k,1)],...
        'YData', [0, 0, z(k,2), z(k,2)],...
        'ZData', [0, 0, 0, z(k,3)]);
    set(body, 'XData', [ctr([1 3],1); NaN; ctr([2 4],1)], ...
        'YData', [ctr([1 3],2); NaN; ctr([2 4],2)],...
        'ZData', [ctr([1 3],3); NaN; ctr([2 4],3)] );

    set(UAV, 'XData', xd(k), 'YData', yd(k), 'ZData', zd(k));

    pause(t(k)-toc);
    pause(0.01);

    % if(k == 1)
    %     gif('quadrotor_capture.gif');
    % elseif(mod(k,10) == 0)
    %     gif('DelayTime',1/800);
    % end
end




clear all;