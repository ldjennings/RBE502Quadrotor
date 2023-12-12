function P = UAV_Trajectory(t)
%UAV_TRAJECTORY Returns the position of the UAV at time t
%   Detailed explanation goes here
    P = [-0.05*(t+3)^2+5;
          -5+.5*t;
        -2*sin(t/8)+0.5*cos(t/2) + 5;];
end

