function [a_commanded] = prop_nav(P_i, V_i, P_t, V_t, N)
    % Calculate the LOS vector
    LOS = P_t - P_i;

    % Calculate the rate of change of LOS
    LOS_dot = (V_t - V_i);  % Assuming continuous tracking

    % Calculate the closing velocity
    V_c = dot((V_i - V_t), LOS) / norm(LOS);

    % Calculate the commanded acceleration
    a_commanded = N * V_c * cross(LOS, cross(LOS, LOS_dot)) / (norm(LOS)^2);
end
