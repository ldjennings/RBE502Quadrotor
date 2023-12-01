clc; clear all; close all;

t0 = 0; tf= 5;
q0 = [0,0,0]; qf = [3,3,3];
qd = zeros(size(q0));

A = quinticpoly(t0, tf, q0, qf, qd, qd, qd, qd)

plotTrajectory(A, t0, tf)

function A = quinticpoly(t0, tf, q0, qf, qd0, qdf, qdd0, qddf)
    M = [1, t0, t0^2, t0^3, t0^4, t0^5;
         0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;
         0, 0, 2, 6*t0, 12*t0^2, 20*t0^3;
         1, tf, tf^2, tf^3, tf^4, tf^5;
         0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
         0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];

    B = [q0; qd0; qdd0; qf; qdf; qddf;];

    A = M \ B;
end

function plotTrajectory(A,t0,tf)
    for i = 1:size(A,2)
        t = linspace(t0,tf,1000);
        q      = A(1,i) + A(2,i) .* t +   A(3,i) .* t.^2 +   A(4,i) .* t.^3 +    A(5,i) .* t.^4 +    A(6,i) .* t.^5;
        qdiff  =        A(2,i)      + 2*A(3,i) .* t    + 3*A(4,i) .* t.^2 +  4*A(5,i) .* t.^3 +  5*A(6,i) .* t.^4;
        qddiff =                    2*A(3,i)         + 6*A(4,i) .* t    + 12*A(5,i) .* t.^2 + 20*A(6,i) .* t.^3;
        
        
        figure, plot(t,q);
        xlabel('time [s]'), ylabel('q [rad]'), title("Joint "+i+" Position");
        
        figure, plot(t,qdiff);
        xlabel('time [s]'), ylabel('qdot [rad]'), title("Joint "+i+" Velocity");
        
        figure, plot(t,qddiff);
        xlabel('time [s]'), ylabel('qddot [rad]'), title("Joint "+i+" Acceleration");
        
    end

end