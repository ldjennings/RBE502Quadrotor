function dz = quadrotor(t, z, u, p, r, n)
% State vector definition
%
%      x1, x2, x3, phi, theta, psi, dx1, dx2, dx3, omega1, omega2, omega3
% z = [z1, z2, z3,  z4,    z5,  z6,  z7,  z8,  z9,    z10,    z11,    z12]
%
% Parameter vector definition
%
%       g,  l,  m, I11, I22, I33, mu, sigma
% p = [p1, p2, p3,  p4,  p5,  p6, p7,    p8]


% Forming the moment of inertia tensor based on the parametr values
I = diag(p(4:6)); 


% Rotation matrix mapping body fixed frame C to inertial frame E
R = [ cos(z(5))*cos(z(6)), sin(z(4))*sin(z(5))*cos(z(6)) - cos(z(4))*sin(z(6)), sin(z(4))*sin(z(6)) + cos(z(4))*sin(z(5))*cos(z(6));
      cos(z(5))*sin(z(6)), cos(z(4))*cos(z(6)) + sin(z(4))*sin(z(5))*sin(z(6)), cos(z(4))*sin(z(5))*sin(z(6)) - sin(z(4))*cos(z(6));
               -sin(z(5)),                                 sin(z(4))*cos(z(5)),                                 cos(z(4))*cos(z(5))];

           

% Adjusting thrust output based on feasible limits
persistent captured;
persistent stabilized;
if isempty(captured)
    captured = false;
    stabilized = false;
end

captured = captured || isCaptured(UAV_Trajectory(t), z);

if stabilized
    fprintf("Hi mom!\n")
elseif captured
    u = stabilize_controller(z,p);
    stabilized = stabilized || all(abs(z(4:12) - zeros(9,1)) < .001);
else
    u = intercept_controller(t, z,p);
end

u = max( min(u, p(7)), 0);


% Computing temporrary variables

% rt = torque vector induced by rotor thrusts
rt = [                   ( u(2) - u(4) )*p(2); 
                         ( u(3) - u(1) )*p(2); 
           ( u(1) - u(2) + u(3) - u(4) )*p(8)];


% Computing time derivative of the state vector
dz(1:3,1) = z(7:9,1);

dz(4:6,1) = [ z(10) + z(12)*cos(z(4))*tan(z(5)) + z(11)*sin(z(4))*tan(z(5));
                                          z(11)*cos(z(4)) - z(12)*sin(z(4));
                              (z(12)*cos(z(4)) + z(11)*sin(z(4)))/cos(z(5))];
                      
dz(7:9,1) = R*([0; 0; sum(u)] + r)/p(3) - [0; 0; p(1)];

dz(10:12,1) = I\( rt + n - cross( z(10:12,1) , I * z(10:12,1) ) ); 
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
    
    %% Defining Controller Gains
    
    z_lambda = .5; z_K = 6; z_n = .5;
    kp = 1;
    kd = 2;

    phi_lambda   = 1.25;          phi_K   =  3;     phi_n   = 1;
    theta_lambda = phi_lambda; theta_K =  phi_K; theta_n = 1;
    psi_lambda   = 1; psi_K   = 3; psi_n   = 1;

    dkine = [xd dxd ddxd;
             yd dyd ddyd;
             zd dzd ddzd;];
    
    kg = [kp kd];

    smc_g = [z_lambda z_K, z_n;
             phi_lambda phi_K phi_n;
             theta_lambda theta_K theta_n;
             psi_lambda psi_K psi_n;];


    u = smc_controller(Z,p,dkine,kg, smc_g);

end

function u = stabilize_controller(Z, p)    
    xd = Z(1); dxd =  0; ddxd = 0;
    yd = Z(2); dyd =  0; ddyd = 0;
    zd = Z(3); dzd =  0; ddzd = 0;
    
    %% Defining Controller Gains
    
    z_lambda = .8; z_K = 6; z_n = 1;
    kp = .3;
    kd = 1;

    phi_lambda   = 2;          phi_K   =  8;     phi_n   = 1;
    theta_lambda = phi_lambda; theta_K =  phi_K; theta_n = phi_n;
    psi_lambda   = 1; psi_K   = 5; psi_n   = 1.5;

    dkine = [xd dxd ddxd;
             yd dyd ddyd;
             zd dzd ddzd;];
    
    kg = [kp kd];

    smc_g = [z_lambda z_K, z_n;
             phi_lambda phi_K phi_n;
             theta_lambda theta_K theta_n;
             psi_lambda psi_K psi_n;];


    u = smc_controller(Z,p,dkine,kg, smc_g);    
end

%     function u = return_controller(t, Z, p)
%     %% Extracting Parameters
%     g = p(1); l = p(2); m = p(3); I11 = p(4); I22 = p(5); I33 = p(6); sigma = p(8);
% 
%     x = 0; dx = Z(7);
%     y = 0; dy = Z(8);
%     z = 0; dz = Z(9);
% 
%     phi = Z(4); theta = Z(5); psi = Z(6);
%     [dphi, dtheta, dpsi] = calcAngV(Z(10:12), Z(4:6));
% 
%     xd = x; dxd =  0; ddxd = 0;
%     yd = y; dyd =  0; ddyd = 0;
%     zd = z; dzd =  0; ddzd = 0;
% 
%     %% Defining Controller Gains
% 
%     z_lambda = .8; z_K = 6; z_n = 1;
%     kp = .3;
%     kd = 1;
% 
%     phi_lambda   = 2;          phi_K   =  8;     phi_n   = 1;
%     theta_lambda = phi_lambda; theta_K =  phi_K; theta_n = phi_n;
%     psi_lambda   = 1; psi_K   = 5; psi_n   = 1.5;
% 
%     %% Linear Position Controls
%     sz = (dzd - dz) + z_lambda*(zd - z);
% 
%     U1 = ((ddzd + g + z_lambda*(zd-z)) + z_K*sz/(abs(sz) + z_n))*m/(cos(phi)*cos(theta));
% 
%     Emax = 1;
%     Fx = m * (ddxd + kd * (dxd - dx) + kp * sat(xd - x,-Emax,Emax));
% 	Fy = m * (ddyd + kd * (dyd - dy) + kp * sat(yd - y,-Emax,Emax));
% 
% 
%     %% Orientation Controls
%     degMax = 35 * pi/180;
% 
%     phid = asin(sat(-Fy / U1,-degMax,degMax));
%     thetad = asin(sat( Fx / U1,-degMax,degMax));
%     psid = 0;
% 
%     ephi   = phid   - phi;
%     etheta = thetad - theta;
%     epsi   = psid   - psi;
% 
% 
%     % Phi controller 
%     sphi = (0 - dphi) + phi_lambda*ephi;
%     U2 = (0 - dtheta*dpsi*(I22-I33)/I11) + phi_lambda*(0 - dphi) + ...
%      phi_K*sphi/(abs(sphi) + phi_n);
% 
%     % Theta controller 
%     stheta = (0 - dtheta) + theta_lambda*etheta;
%     U3 = (0 + dphi*dpsi*(I11-I33)/I22) + theta_lambda*(0 - dtheta) + ...
%          theta_K*stheta/(abs(stheta) + theta_n);
% 
%     % Psi controller
%     spsi = (0 - dpsi) + psi_lambda*epsi;
%     U4 = (0 - dphi*dtheta*(I11-I22)/I33) + psi_lambda*(0 - dpsi) + ...
%         + psi_K*spsi/(abs(spsi) + psi_n);
% 
%     U = [U1 U2 U3 U4]';
% 
% 
%     %% Calculating individual control inputs
%     length_11 = l/I11;
%     length_22 = l/I22;
%     sigma_33 = sigma/I33;
%     A = [         1         1         1          1;
%                   0 length_11         0 -length_11;
%          -length_22         0 length_22          0;
%            sigma_33 -sigma_33  sigma_33 -sigma_33;];
% 
%     u = A\U;
% end


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

function y = sat(x, lowerbound, upperbound)
    y = max(min(x, upperbound), lowerbound);
end