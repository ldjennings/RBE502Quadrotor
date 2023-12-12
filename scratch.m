clc; clear all; close all;




t0 = 0; tf = 10; 
t = linspace(0,tf, tf*200);

Z = zeros(3, length(t));
Zp = zeros(3, length(t));

V = zeros(3, length(t));
Vp = zeros(3, length(t));

A = zeros(3, length(t));
Ap = zeros(3, length(t));

for i = 1:length(t)
    [z, v, a] = traj(t(i));
    Z(:,i) = z;
    V(:,i) = v;
    A(:,i) = a;
end

figure;
plot(t, Z)
title("positions")
figure;
plot(t, V)
title("Velocities")
figure;
plot(t, A)
title("Accelerations")

Zp(:,1) = Z(:,1);
Vp(:,1) = V(:,1);
Ap(:,1) = A(:,1);
for i = 2:length(t)
    [p, vp, ap] = predictNextState(Z(:,i-1));
    Zp(:,i) = p;
    Vp(:,i) = vp;
    Ap(:,i) = ap;
end

figure;
plot(t, Zp)
title("Predicted positions")
figure;
plot(t, Vp)
title("Predicted Velocities")
figure;
plot(t, Ap)
title("Predicted Accelerations")

function [z, v, a] = traj(t)
    z = [.5*cos(t/2);
         .5*sin(t/2);
            2 + t/10;];
    v = [-sin(t/2)/4;
         cos(t/2)/4;
            1/10;];
    a = [-cos(t/2)/8;
         -sin(t/2)/8;
                   0];
end


function [p, v, a] = predictNextState(P1)
    persistent states;
    if isempty(states)
        states = repmat(P1, 1, 5); % Initialize with 5 copies of P1 for each dimension
    end

    % Update the states
    states(:, 2:end) = states(:, 1:end-1); % Shift left
    states(:, 1) = P1; % Add new state at the beginning

    dt = 1/200;  % Time step
    t = (-4*dt):dt:0;  % Time vector for the states (negative to positive)

    % Initialize vectors for velocity and acceleration
    v = zeros(3,1);
    a = zeros(3,1);
    p = zeros(3,1);

    % Fit a quartic polynomial and calculate derivatives for each dimension
    for dim = 1:3
        % Polynomial fitting for each dimension
        coefficients = polyfit(t, states(dim, :), 4);

        % Coefficients of the derivative polynomials for velocity and acceleration
        v_coefficients = [4*coefficients(1), 3*coefficients(2), 2*coefficients(3), coefficients(4)];
        a_coefficients = [3*v_coefficients(1), 2*v_coefficients(2), v_coefficients(3)];

        % Evaluate velocity and acceleration at the current time point (t=0)
        v(dim) = polyval(v_coefficients, 0);
        a(dim) = polyval(a_coefficients, 0);

        % Predict the next position for each dimension
        p(dim) = polyval(coefficients, dt);
    end
end





function [p, v, a] = predictNextStateold(P1)
    persistent previous2;
    if isempty(previous2)
        fprintf("I'm empty!\n")
        previous2 = repmat(P1, 1, 2);
    end

    dt = 1/200;
    V = (P1 - previous2(:,2))/dt;
    % a = (V - (previous2(:,2) - previous2(:,1)) / dt) / dt;
    a = (P1 - 2*previous2(:,1) + previous2(:,2))/dt^2;

    p = P1 + V * dt + .5 * a * dt^2;
    v = V + a*dt;
    
    % Updating the previous two states of the robot
    previous2(:,2) = previous2(:,1);
    previous2(:,1) = P1;

end