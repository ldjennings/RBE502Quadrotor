classdef quadrotor2
    properties
        m
        g

        c_z
        e_z
        n_z

        c_yaw
        e_yaw
        n_yaw
        %K_3
    end

    methods
        %% Fully Actuated Systems

        % We should have information on the state vecotor Z defined as:
        % Z = [x, y, z, roll, pitch, yaw, x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot]
        % Z_dot = [x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot, x_ddot, y_ddot, z_ddot, roll_ddot, pitch_ddot, yaw_ddot]

        function u = ZController(self, Z, z_desired)
            % Make a controller that allows the quadrotor to get to the desiried z
            % position

            % Z [1x12 State Vector] The state of the quadrotor
            % Z_dot [1x12 State Vector] The deriviative of the state of the quadrotor
            % z_desired [1x3 Vector] Represents the z desired [position, velocity, acceleration]

            
            % This should be the disterbances in the system
            d_z = 0;  % Assuming no distirbances, it should be zero
            %d_z = self.K_3*z_dot/self.m; 

            z = Z(3);
            z_dot = Z(9);

            s_z = self.sliding_surface(z, z_desired(1), z_dot, z_desired(2), 1);

            u = self.m*(self.c_z*(z_desired(2) - z_dot) + z_desired(3) + self.g + d_z + self.s_dot(s_z, self.e_z, self.n_z) ) / (cos(roll)*cos(pitch));
        end

        function u = YawContoller(self, Z, yaw_desired)
            % Make a controller that allows the quadrotor to rotate to the desired
            % Yaw angle (rotation about the z axis)

            % Z [1x12 State Vector] The state of the quadrotor
            % yaw_desired [1x3 Vector] Represents the yaw desired [position, velocity, acceleration]

            % This should be the disterbances in the system
            d_yaw = 0;  % Assuming no distirbances
            %d_yaw = self.K_6*r/I_z; 

            yaw = Z(6);
            yaw_dot = Z(12);

            s_yaw = self.sliding_surface(yaw, yaw_desired(1), yaw_dot, yaw_desired(2), 1);

            u = (I_z/C)*(self.c_yaw(yaw_desired(2) - yaw_dot) + yaw_desired(3) + d_yaw + self.s_dot(s_yaw, self.e_yaw, self.n_yaw));
        end
        
        %% Underactuated Controllers
        
        function u = XController()
            % Make a controller that helps move the robot to the desired x location
        end
        
        function u = YController()
            % Make a controller that helps move the robot to the desird y location
        end

        %% Supporting Functions

        function s = sliding_surface(self, a, a_desired, a_dot, a_desired_dot, c)
            % The equation represented the sliding manifold for the fully actuated
            % system, a being the placeholder for z -> z_desired and yaw ->
            % yaw_desired
            
            % a                 [float] z or yaw
            % a_desired         [float] desired goal
            % a_dot             [float] the velocty 
            % a_desired_dot     [float] the desired velocity
        
            % c                 [const float] Some tunable value, where c > 0. 
            
            s = c*(a_desired - a) + (a_desired_dot - a_dot);
        end

        function sd = s_dot(self, s, e, n)
            % Represents another definition for the deriviative of our sliding surface
            % This works in conjunction with our other definition of s_dot
            
            % s [value] our evaluated sliding surface
            % e [const float] ##TODO Figure out what that actually means for this
            %                   quadrotor
            % n [const float] ##TODO Figure out what that actually means for this
            %                   quadrotor
        
            sd = - e*sign(s) - n*s;
        end
    end
end


function quadrotor()
    % Make a quadrotor system that spits out the states

end



%% Supporting Function Implementations

% function sd = s_dot(a_dot, a_desired_dot, a_ddot, a_desired_ddot, c)
%     % Representation of s_dot based on the time derivative of the sliding
%     % manifold s, where s was:
%     %   s = c*(a_desired - a) + (a_desired_dot - a_dot);
% 
%     % a_dot             [float] the velocty 
%     % a_desired_dot     [float] the desired velocity
%     % a_ddot            [float] the acceleration
%     % a_desired_ddot    [float] the desired acceleration
% 
%     % c                 [const float] Some tunable value, where c > 0. 
% 
%     sd = c*(a_desired_dot - a_dot) + (a_desired_ddot - a_ddot)
% 
% end







