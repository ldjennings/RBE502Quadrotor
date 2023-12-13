function [V, A] = UAV_derivatives(t)

    V = [ - t/10 - 3/10;
                    1/2;
          - cos(t/8)/4 - sin(t/2)/4;];
    A = [- 1/10;
              0;
         sin(t/8)/32 - cos(t/2)/8;];

end