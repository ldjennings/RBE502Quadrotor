function u = quadrotor_controller(t,Z,p)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

persistent currentState;
if isempty(currentState)
    currentState = UAVState.Intercept;
end

switch currentState
    case UAVState.Intercept
        u = intercept_controller(t, Z, p);
        if isCaptured(UAV_Trajectory(t), Z)
            currentState = UAVState.Stabilize;
        end

    case UAVState.Stabilize
        u = stabilize_controller(Z, p);
        if all(abs(Z(4:12) - zeros(9,1)) < .05)
            currentState = UAVState.Return;
        end

    case UAVState.Return
        u = return_controller(Z, p);

    otherwise
        error('Unknown state');
end
end

function u = intercept_controller(t, Z, p)
    %% Defining desired kinematics
    dt = 1/200;
    yt_n = UAV_Trajectory(t);
    yt_1 = UAV_Trajectory(t-dt);
    yt_2 = UAV_Trajectory(t-2*dt);
    
    yt = [yt_2 yt_1 yt_n];
    
    if t > dt * 4
        [Pnext, V, A] = predictNextState(yt);

        xd = Pnext(1); dxd =  V(1); ddxd = A(1);
        yd = Pnext(2); dyd =  V(2); ddyd = A(2);
        zd = Pnext(3); dzd =  V(3); ddzd = A(3);
    else
        xd = Z(1); dxd =  0; ddxd = 0;
        yd = Z(2); dyd =  0; ddyd = 0;
        zd = Z(3); dzd =  0; ddzd = 0;
    end  
    
    
    z_lambda = .5; z_K = 6; z_n = .5;
    kp = 1.5;
    kd = 2;

    phi_lambda   = 1.25;          phi_K   =  3;     phi_n   = 1;
    theta_lambda = phi_lambda; theta_K =  phi_K; theta_n = 1;
    psi_lambda   = 1; psi_K   = 3; psi_n   = 1;

    packageVariables;

    u = smc_controller(Z,p,dkine,kg, smc_g);

end

function u = stabilize_controller(Z, p)    
    xd = Z(1); dxd =  0; ddxd = 0;
    yd = Z(2); dyd =  0; ddyd = 0;
    zd = Z(3); dzd =  0; ddzd = 0;
    
    
    z_lambda = .8; z_K = 6; z_n = 1;
    kp = .3;
    kd = 1;

    phi_lambda   = 2;          phi_K   =  8;     phi_n   = 1;
    theta_lambda = phi_lambda; theta_K =  phi_K; theta_n = phi_n;
    psi_lambda   = 1; psi_K   = 5; psi_n   = 1.5;

    packageVariables;

    u = smc_controller(Z,p,dkine,kg, smc_g);    
end

    function u = return_controller(Z, p)
    xd = 0; dxd =  0; ddxd = 0;
    yd = 0; dyd =  0; ddyd = 0;
    zd = 0; dzd =  0; ddzd = 0;


    z_lambda = .8; z_K = 6; z_n = 1;
    kp = .3;
    kd = 1;

    phi_lambda   = 2;          phi_K   =  8;     phi_n   = 1;
    theta_lambda = phi_lambda; theta_K =  phi_K; theta_n = phi_n;
    psi_lambda   = 1; psi_K   = 5; psi_n   = 1.5;


    packageVariables;

    u = smc_controller(Z,p,dkine,kg, smc_g);  

end


function [p, v, a] = predictNextState(Prev3)
    P1 = Prev3(:,3); P2 = Prev3(:,2); P3 = Prev3(:,1);
    dt = 1/200;
    V = (P1 - P3)/(2*dt);
    a = (P1 - 2*P2 + P3)/dt^2;

    p = P1 + V * dt + .5 * a * dt^2;
    v = V + a*dt; 
end

function b = isCaptured(zd, z)
    p = z(1:3); pd = zd(1:3);
    b = norm(p-pd,2) <= .1;
end