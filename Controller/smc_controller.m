function u = smc_controller(Z, p, desired_kinematics, k_gains, smc_gains)
%SMC_CONTROLLER Summary of this function goes here
%   Detailed explanation goes here
    %% Extracting Parameters
    g = p(1); l = p(2); m = p(3); I11 = p(4); I22 = p(5); I33 = p(6); sigma = p(8);

    x = Z(1); dx = Z(7);
    y = Z(2); dy = Z(8);
    z = Z(3); dz = Z(9);
    
    phi = Z(4); theta = Z(5); psi = Z(6);
    [dphi, dtheta, dpsi] = calcAngV(Z(10:12), Z(4:6));

    xd = desired_kinematics(1, 1); dxd = desired_kinematics(1, 2); ddxd = desired_kinematics(1, 3);
    yd = desired_kinematics(2, 1); dyd = desired_kinematics(2, 2); ddyd = desired_kinematics(2, 3);
    zd = desired_kinematics(3, 1); dzd = desired_kinematics(3, 2); ddzd = desired_kinematics(3, 3);
 
    
    
    %% Defining Controller Gains

    kp = k_gains(1);
    kd = k_gains(2);
    
    z_lambda = smc_gains(1, 1); z_K = smc_gains(1, 2); z_n = smc_gains(1, 3);
    phi_lambda = smc_gains(2, 1); phi_K = smc_gains(2, 2); phi_n = smc_gains(2, 3);
    theta_lambda = phi_lambda; theta_K = phi_K; theta_n = phi_n;
    psi_lambda = smc_gains(4, 1); psi_K = smc_gains(4, 2); psi_n = smc_gains(4, 3);

    %% Linear Position Controls
    sz = (dzd - dz) + z_lambda*(zd - z);

    U1 = ((ddzd + g + z_lambda*(zd-z)) + z_K*sz/(abs(sz) + z_n))*m/(cos(phi)*cos(theta));

    Emax = 1.5;
    Fx = m * (ddxd + kd * (dxd - dx) + kp * sat(xd - x,-Emax,Emax));
	Fy = m * (ddyd + kd * (dyd - dy) + kp * sat(yd - y,-Emax,Emax));

    
    %% Orientation Controls
    degMax = 35 * pi/180;

    phid = asin(sat(-Fy / U1,-degMax,degMax));
    thetad = asin(sat( Fx / U1,-degMax,degMax));
    psid = 0;

    ephi   = phid   - phi;
    etheta = thetad - theta;
    epsi   = psid   - psi;


    % Phi controller 
    sphi = (0 - dphi) + phi_lambda*ephi;
    U2 = (0 - dtheta*dpsi*(I22-I33)/I11) + phi_lambda*(0 - dphi) + ...
     phi_K*sphi/(abs(sphi) + phi_n);

    % Theta controller 
    stheta = (0 - dtheta) + theta_lambda*etheta;
    U3 = (0 + dphi*dpsi*(I11-I33)/I22) + theta_lambda*(0 - dtheta) + ...
         theta_K*stheta/(abs(stheta) + theta_n);

    spsi = (0 - dpsi) + psi_lambda*epsi;
    U4 = (0 - dphi*dtheta*(I11-I22)/I33) + psi_lambda*(0 - dpsi) + ...
        + psi_K*spsi/(abs(spsi) + psi_n);

    U = [U1 U2 U3 U4]';
    

    %% Calculating individual control inputs
    length_11 = l/I11;
    length_22 = l/I22;
    sigma_33 = sigma/I33;
    A = [         1         1         1          1;
                  0 length_11         0 -length_11;
         -length_22         0 length_22          0;
           sigma_33 -sigma_33  sigma_33 -sigma_33;];

    u = A\U;

end

function [dphi, dtheta, dpsi] = calcAngV(W, alpha)
    phi = alpha(1); theta = alpha(2); 
    T = [1 0 -sin(theta);
         0 cos(phi) sin(phi)*cos(theta);
         0 -sin(phi) cos(theta)*cos(theta)];
    aDot = T\W;
    dphi = aDot(1); dtheta = aDot(2); dpsi = aDot(3);
end

function y = sat(x, lowerbound, upperbound)
    y = max(min(x, upperbound), lowerbound);
end
