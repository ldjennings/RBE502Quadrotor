%% Goal - Create a class that is able to work with ODE
% For that to work, the quadrotor needs to be able to compute z_dot. Z_dot
% multiplied by time will describe the change in the state z.... 


% if im not missunderstanding, the simulation should take the change in the
% state z_dot, multiply it by some delta_time, and that would cause a new
% change a new change in z. Based on the state, I also assume that we need
% to update the input u in the same step. So tldr we need to 

% be able to output the change in state z_dot given state and input
% variables

% have something that can update the input U



classdef quadrotor2
    properties
        % Properties of our quadrotor
        m           % mass
        g = 9.81    % acceleration due to gravity
        l           % length of links in the quadrotor
        Jr          % inertia of the z-axis

        Ixx         % X of the inertia matrix
        Iyy         % Y of the inertia matrix
        Izz         % Z of the inertia matrix
        
        b           % Const based on thrust generated by rotors -> F = b(omega)^2      
        k           % Const based on rotor drag M = -k(omega)^2
        C           % Proportional coefficient for yaw torque M_yaw = C(F1 - F2 + F3 -F4)

        e = zeros(4,1);
        n = zeros(4,1);
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
        function runSim(self)
            % Is able to run a simulation of the quadrotor assuming that we
            % have a desired position we want to move to 

            duration = 10;
            delta_time = 0.2;
            time_steps = duration/delta_time;

            dZ = zeros(12, 1); 

            % x_init
            Z = zeros(12, 1);
            % u_init
            omega = self.CalcOmega(desired);

            for step = 0:time_steps
                % Sub the dynamics in, get the change in z, z_dot
                dZ = self.CalcDynamics(Z,omega);

                % Calc the new z
                Z = Z + dZ*delta_time;

                % From the z, calc the new u
                % u = GetU(z)

                disp(step)
                % Store info (z, z_dot, u, time_step)
            end

        end
        function dZ = CalcDynamics(self, Z, omega)
            % Outputs the change in the state

            U = self.RotorVelocityToU(omega);

            Ur = U(5);

            roll = Z(4);
            pitch = Z(5);
            yaw = Z(6);

            p = Z(10); %roll_dot
            q = Z(11); %picth_dot
            r = Z(12); %yaw_dot

            % Linear And Angular Velocity
            dZ(1:3) = Z(7:9);
            dZ(4:6) = [1, sin(roll)*tan(pitch), cos(roll)*tan(pitch);
                        0, cos(roll),            -sin(roll);
                        0, sin(roll)*sec(pitch), cos(roll)*sec(pitch)]*Z(10:12);

            % Linear And Angular Accelerations
            x_ddot = 1/self.m*(cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw))*U(1);
            y_ddot = 1/self.m*(cos(roll)*sin(pitch)*sin(yaw) + sin(roll)*cos(yaw))*U(1);
            z_ddot = 1/self.m*(cos(roll)*cos(pitch))*U(1) - self.g;

            roll_ddot = q*r*((self.Iyy -self.Izz)/self.Ixx) + self.Jr/self.Ixx*q*Ur + self.l/self.Ixx*U(2);
            pitch_ddot = p*r*((self.Izz -self.Ixx)/self.Iyy) - self.Jr/self.Iyy*p*Ur + self.l/self.Iyy*U(3);
            yaw_ddot = p*q*((sefl.Ixx -self.Iyy)/self.Ixx) + self.C/self.Izz*U(4);

            dZ(7:12) = [x_ddot, y_ddot, z_ddot, roll_ddot, pitch_ddot, yaw_ddot]';
        end

        function U = RotorVelocityToU(self, omega)
            % Conversion of the rotor velocitys omega_i, i = 1,2,3,4 to
            % "Control Inputs" U_i, and U_5, the residual angular velocity

            d = self.l*self.b;
            
            U(1:4) =  [self.b,self.b,self.b,self.b;
                       d, 0, -d, 0;
                       0, -d, 0, d;
                       -self.k,self.k,-self.k,self.k;]*omega.^2;

            U(5) = -omega(1) + omega(2) -omega(3) + omega(4);
        end

        function omega = CalcOmega(self, Z, desired)
            % Gets the needed rotor speeds Omega_i, i = 1,2,3,4
            % Desired is composed of the desired [Pose, Pose_dot,
            % Pose_ddot], where Pose is [x, y, z, roll, pitch, yaw]

            U = zeros(4, 1);
            U(1) = self.ZController(Z, [desired(3), desired(9), desired(15)]);
            U(4) = self.ZController(Z, [desired(3), desired(9), desired(15)]);

        end

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







