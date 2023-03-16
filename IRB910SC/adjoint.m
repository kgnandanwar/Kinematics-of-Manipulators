function twist_inB = adjoint(twist_inA,T_AB)
    R = T_AB(1:3, 1:3);
    p = T_AB(1:3, 4);
    o = zeros(3);
    skewp = [0 -p(3) p(2) ; p(3) 0 -p(1) ; -p(2) p(1) 0 ];
    Adj = [R o ; skewp*R R];
    twist_inB = Adj*twist_inA;
end