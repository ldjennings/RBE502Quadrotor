%% Goal - Create a class that is able to work with ODE
% For that to work, the quadrotor needs to be able to compute z_dot. Z_dot
% multiplied by time will describe the change in the state z.... 



classdef quadrotor2 < handle
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

        Ur = 0;     % DO NOT TOUCH Something that is important for running the controller
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
            self.Ur = 0;
            omega = self.CalcOmega(init_omega, desired);

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

            self.Ur = U(5);

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

            roll_ddot = q*r*((self.Iyy -self.Izz)/self.Ixx) + self.Jr/self.Ixx*q*self.Ur + self.l/self.Ixx*U(2);
            pitch_ddot = p*r*((self.Izz -self.Ixx)/self.Iyy) - self.Jr/self.Iyy*p*self.Ur + self.l/self.Iyy*U(3);
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

        function omega = UToRotorVelocity(self, U)
            % Conversion of the Control Inputs U to Rotor Speeds omega

            d = self.l*self.b;
            
            omega =  ([self.b,self.b,self.b,self.b;
                       d, 0, -d, 0;
                       0, -d, 0, d;
                       -self.k,self.k,-self.k,self.k;]\U(1:4)).^(1/2);
        end

        function Ur = GetUr(omega)
            Ur = -omega(1) + omega(2) -omega(3) + omega(4);
        end

        function [omega] = CalcOmega(self, Z, Z_dot, desired)
            % Gets the needed rotor speeds Omega_i, i = 1,2,3,4
            % Desired is composed of the desired [Pose, Pose_dot,
            % Pose_ddot], where Pose is [x, y, z, roll, pitch, yaw]

            U = zeros(4, 1);
            U(1) = self.ZController(Z, Z_dot, desired);
            U(4) = self.YawController(Z, Z_dot, desired);

            U(2) = self.XPitchController(Z, Z_dot, desired, U(1), self.Ur);
            U(3) = self.YRollController(Z, Z_dot, desired, U(1), self.Ur);
            
            omega = self.UToRotorVelocity(U);
            
        end

        function u = ZController(self, Z, Z_dot, desired)
            % Make a controller that allows the quadrotor to get to the desiried z
            % position

            % Z [1x12 State Vector] The state of the quadrotor
            % Z_dot [1x12 State Vector] The deriviative of the state of the quadrotor
            % z_desired [1x3 Vector] Represents the z desired [position, velocity, acceleration]

            % This should be the disterbances in the system
            d_z = 0;  % Assuming no distirbances, it should be zero
            %d_z = self.K_3*z_dot/self.m; 

            [~, ~, z, roll, pitch, ~, ~, ~, z_dot, ~, ~, ~, ~, ~, ~, ~, ~, ~] = self.DecomposeZ(Z, Z_dot);
            [~, ~, z_desired, ~, ~, ~, ~, ~, z_desired_dot, ~, ~, ~, ~, ~, z_desired_ddot, ~, ~, ~] = self.DecomposeDesired(desired);

            s_z = self.sliding_surface(z, z_desired, z_dot, z_desired_dot, 1);

            u = self.m*(self.c_z*(z_desired_dot - z_dot) + z_desired_ddot - self.g + d_z + self.s_dot(s_z, self.e_z, self.n_z) ) / (cos(roll)*cos(pitch));
        end

        function u = YawController(self, Z, Z_dot, desired)
            % Make a controller that allows the quadrotor to rotate to the desired
            % Yaw angle (rotation about the z axis)

            % Z [1x12 State Vector] The state of the quadrotor
            % yaw_desired [1x3 Vector] Represents the yaw desired [position, velocity, acceleration]

            % This should be the disterbances in the system
            d_yaw = 0;  % Assuming no distirbances
            %d_yaw = self.K_6*r/I_z; 

            [~, ~, ~, ~, ~, yaw, ~, ~, ~, ~, ~, yaw_dot, ~, ~, ~, ~, ~, ~] = self.DecomposeZ(Z, Z_dot);
            [~, ~, ~, ~, ~, yaw_desired, ~, ~, ~, ~, ~, yaw_desired_dot, ~, ~, ~, ~, ~, yaw_desired_ddot] = self.DecomposeDesired(desired);

            s_yaw = self.sliding_surface(yaw, yaw_desired, yaw_dot, yaw_desired_dot, 1);

            u = (self.Izz/self.C)*(self.c_yaw(yaw_desired_dot - yaw_dot) + yaw_desired_ddot + d_yaw - self.s_dot(s_yaw, self.e_yaw, self.n_yaw));
        end
        
        %% Underactuated Controllers
        
        function u = XPitchController(self, Z, Z_dot, desired, U1, Ur)
            % Make a controller that helps move the robot to the desired x, pictch location
            
            [x, ~, ~, roll, pitch, yaw, x_dot, ~, ~, ~, pitch_dot, ~, x_ddot, ~, ~, ~, ~, ~] = self.DecomposeZ(Z, Z_dot);
            [x_desired, ~, ~, ~, pitch_desired, ~, x_desired_dot, ~, ~, ~, pitch_desired_dot, ~, x_desired_ddot, ~, ~, ~, pitch_desired_ddot, ~] = self.DecomposeDesired(desired);

            c1 = 11*self.m/(U1*cos(roll)*cos(yaw));
            c2 = 6*self.m/(U1*cos(roll)*cos(yaw));
            c3 = 1;
            c4 = 6;

            d3 = -p*r*(self.Izz -self.Ixx)/self.Iyy + self.Jr*p*Ur/self.Iyy;

            s = c1(x_desired_dot - x_dot) + c2(x_desired - x) + c3(pitch_desired_dot - pitch_dot) + c4(pitch_desired - pitch);
            u = self.Iyy/self.l *(c1/c3*(x_desired_ddot - x_ddot) +c2/c3*(x_desired_dot - x_dot) + pitch_desired_ddot + c4/c3*(pitch_desired_dot - pitch_dot) + d3 - 1/c3*self.s_dot(s, self.e(3), self.n(3)));


        end
        
        function u = YRollController(self, Z, Z_dot, desired, U1, Ur)
            % Make a controller that helps move the robot to the desird y location

            [~, y, ~, roll, ~, yaw, ~, y_dot, ~, roll_dot, ~, ~, ~, y_ddot, ~, ~, ~, ~] = self.DecomposeZ(Z, Z_dot);
            [~, y_desired, ~, roll_desired, ~, ~, ~, y_desired_dot, ~, roll_desired_dot, ~, ~, ~, y_desired_ddot, ~, roll_desired_ddot, ~, ~] = self.DecomposeDesired(desired);

            c5 = -11*self.m/(U1*cos(yaw));
            c6 = -6*self.m/(U1*cos(yaw));
            c7 = 1;
            c8 = 6;

            d4 = -q*r(self.Iyy - self.Izz)/self.Ixx - self.Jr*q*Ur/self.Ixx;

            s = c5(y_desired_dot - y_dot) + c6(y_desired - y) + c7(roll_desired_dot - roll_dot) + c8(roll_desired - roll);
            u = self.Ixx/self.l*(c5/c7*(y_desired_ddot - y_ddot) +c6/c7*(y_desired_dot - y_dot) + roll_desired_ddot + c8/c7*(roll_desired_dot - roll_dot) + d4 - 1/c7*self.s_dot(s, self.e(4), self.n(4)));
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
 
        function [x, y, z, roll, pitch, yaw, x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot, x_ddot, y_ddot, z_ddot, roll_ddot, pitch_ddot, yaw_ddot] = DecomposeZ(self, Z, Z_dot)
            x = Z(1);
            y = Z(2);
            z = Z(3);
            roll = Z(4);
            pitch = Z(5);
            yaw = Z(6);

            x_dot = Z(7);
            y_dot = Z(8);
            z_dot = Z(9);
            roll_dot = Z(10);
            pitch_dot = Z(11);
            yaw_dot = Z(12);

            x_ddot = Z_dot(7);
            y_ddot = Z_dot(8);
            z_ddot = Z_dot(9);
            roll_ddot = Z_dot(10);
            pitch_ddot = Z_dot(11);
            yaw_ddot = Z_dot(12);
        end

        function [x_desired, y_desired, z_desired, roll_desired, pitch_desired, yaw_desired, x_desired_dot, y_desired_dot, z_desired_dot, roll_desired_dot, pitch_desired_dot, yaw_desired_dot, x_desired_ddot, y_desired_ddot, z_desired_ddot, roll_desired_ddot, pitch_desired_ddot, yaw_desired_ddot] = DecomposeDesired(self, d)
            x_desired = d(1);
            y_desired = d(2);
            z_desired = d(3);
            roll_desired = d(4);
            pitch_desired = d(5);
            yaw_desired = d(6);

            x_desired_dot = d(7);
            y_desired_dot = d(8);
            z_desired_dot = d(9);
            roll_desired_dot = d(10);
            pitch_desired_dot = d(11);
            yaw_desired_dot = d(12);

            x_desired_ddot = d(13);
            y_desired_ddot = d(14);
            z_desired_ddot = d(15);
            roll_desired_ddot = d(16);
            pitch_desired_ddot = d(17);
            yaw_desired_ddot = d(18);
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







