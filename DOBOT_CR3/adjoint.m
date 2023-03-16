function twist_inB = adjoint(twist_inA,T_AB)
    twist_inB = [T_AB(1:3,1:3)                   zeros(3,3);
                 skew(T_AB(1:3,4))*T_AB(1:3,1:3) T_AB(1:3,1:3)] * twist_inA;
end

