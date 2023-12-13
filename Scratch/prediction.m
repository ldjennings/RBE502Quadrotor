clc; clear all; close all;
clear predictNextState;

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
tiledlayout(3, 3);
nexttile;
plot(t, Z)
title("positions")
nexttile;
plot(t, V)
title("Velocities")
nexttile;
plot(t, A)
title("Accelerations")

Zp(:,1) = Z(:,1);
Vp(:,1) = V(:,1);
Ap(:,1) = A(:,1);
Prev3 = repmat(Z(:,i-1), 1, 3);
for i = 2:length(t)
    Prev3 = [Prev3(:,2:end) Z(:,i-1)];
    [p, vp, ap] = predictNextState(Prev3);
    Zp(:,i) = p;
    Vp(:,i) = vp;
    Ap(:,i) = ap;
end

nexttile;
startGraph = 4;
plot(t(:,startGraph:end), Zp(:,startGraph:end))
title("Predicted positions")
nexttile;
plot(t(:,startGraph:end), Vp(:,startGraph:end))
title("Predicted Velocities")
nexttile;
plot(t(:,startGraph:end), Ap(:,startGraph:end))
title("Predicted Accelerations")

diffZ = (Z-Zp);
diffV = (V-Vp);
diffA = (A-Ap);

nexttile;
plot(t(:,startGraph:end), diffZ(:,startGraph:end))
title("D positions")
nexttile;
plot(t(:,startGraph:end), diffV(:,startGraph:end))
title("D Velocities")
nexttile;
plot(t(:,startGraph:end), diffA(:,startGraph:end))
title("D Accelerations")

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


function [p, v, a] = predictNextState(Prev3)
    P1 = Prev3(:,3); P2 = Prev3(:,2); P3 = Prev3(:,1);
    dt = 1/200;
    V = (P1 - P3)/(2*dt);
    a = (P1 - 2*P2 + P3)/dt^2;

    p = P1 + V * dt + .5 * a * dt^2;
    v = V + a*dt; 
end