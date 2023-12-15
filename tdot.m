function Tdot = tdot(phi, theta, dphi, dtheta)
%TDOT Summary of this function goes here
%   Detailed explanation goes here
    % phi = P(1); theta = P(2);
    % dphi = AV(1); dtheta = AV(2);

    Tdot = [
    0              0                                     -dtheta*cos(theta);
    0 -dphi*sin(phi)  dphi*cos(phi)*cos(theta) - dtheta*sin(phi)*sin(theta);
    0 -dphi*cos(phi) -dphi*cos(theta)*sin(phi) - dtheta*cos(phi)*sin(theta);];
    


end

