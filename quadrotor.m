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
u = controller(t, z,p);
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

function u = controller(t, Z, p)
    % Trajectory
    % zd =    2 + t/10; z = Z(3); dzd =         1/10; dz = Z(9); ddzd =           0;
    % yd = .5*sin(t/2); y = Z(2); dyd =   cos(t/2)/4; dy = Z(8); ddyd = -sin(t/2)/8;
    % xd = .5*cos(t/2); x = Z(1); dxd =  -sin(t/2)/4; dx = Z(7); ddxd = -cos(t/2)/8;
    zd = 1; z = Z(3); dzd = 0; dz = Z(9); ddzd = 0;
    yd = 0; y = Z(2); dyd = 0; dy = Z(8); ddyd = 0;
    xd = 0; x = Z(1); dxd = 0; dx = Z(7); ddxd = 0;
    
    phi = Z(4); theta = Z(5); psi = Z(6);
    % w1 = Z(10); w2 = Z(11);   w3 = Z(12);
    W = Z(10:12);

    T = [1 0 -sin(theta);
         0 cos(phi) sin(phi)*cos(theta);
         0 -sin(phi) cos(theta)*cos(theta)];
    aDot = T\W;
    dphi = aDot(1); dtheta = aDot(2); dpsi = aDot(3);

    g = p(1); l = p(2); m = p(3); I11 = p(4); I22 = p(5); I33 = p(6); sigma = p(8);

    z_lambda = 1; z_K = 1; z_n = 1;
    kp = 1;
    kd = 1;

    phi_lambda   = .5; phi_K   =  1; phi_n   = 1;
    theta_lambda = .5; theta_K =  1; theta_n = 1;
    psi_lambda   = 1; psi_K   = 1; psi_n   = 1;
    % phi_lambda   = 5; phi_K   = 100; phi_n   = .3;
    % theta_lambda = 5; theta_K = 100; theta_n = .3;
    % psi_lambda   =  5; psi_K   =  100; psi_n   = .3;

    sz = (dzd - dz) + z_lambda*(zd - z);

    U1 = (ddzd + g + z_lambda*(zd-z))*m/(cos(phi)*cos(theta)) + z_K*sz/(abs(sz) + z_n);


    Fx = m * (ddxd + kd * (dxd - dx) + kp * sat(xd - x,-1,1));
	Fy = m * (ddyd + kd * (dyd - dy) + kp * sat(yd - y,-1,1));


    degMax = 20 *pi/180;

    phid = asin(sat(-Fy / U1,-degMax,degMax));
    thetad = asin(sat( Fx / U1,-degMax,degMax));
    psid = 0;

    


    ephi   = phid   - phi;
    etheta = thetad - theta;
    epsi   = psid   - psi;


    % esd = [phid thetad psid];
    % esb = [phi theta psi];
    % ebd = space_to_body(esb, esd);
    % 
    % ephi = wrapToPi(0 - ebd(1));
    % etheta = wrapToPi(0 - ebd(2));
    % epsi = wrapToPi(0 - ebd(3));



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



    length_11 = l/I11;
    length_22 = l/I22;
    sigma_33 = sigma/I33;
    A = [         1         1         1          1;
                  0 length_11         0 -length_11;
         -length_22         0 length_22          0;
           sigma_33 -sigma_33  sigma_33 -sigma_33;];

    u = A\U;

    % u = repmat(U1/4,4,1)
end


function ebd = space_to_body(esb, esd)
    % Rsb = intrinsic_zyx(esb);
    % Rsd = extrinsic_xyz(esd);
    Rsb = eul2rotm(esb);
    Rsd = extrinsic_xyz(esd);

    Rbd = Rsb\Rsd;

    ebd = rotm2eul(Rbd,"XYZ")';
    
end


function R = intrinsic_zyx(ang)
    phi = ang(1); theta = ang(2);psi = ang(3);
    R = rotz(psi) * roty(theta) * rotx(phi);
end

function R = extrinsic_xyz(ang)
    phi = ang(1); theta = ang(2);psi = ang(3);
    R = rotx(phi)  * roty(theta) * rotz(psi);
end

function y = sat(x, lowerbound, upperbound)
    y = max(min(x, upperbound), lowerbound);
end