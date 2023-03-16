function J_a = jacoba(S,M,q)    
       
%{
    jacoba: computes the analytic jacobian of the robotic arm

    Inputs: S - screw axis matrix
            q - joint variables
            M - Home configuration
    Output: J_a - Analytic Jacobian
%}
    
    % your code here
    
    s = length(q);
    T = fkine(S, M,q);
    pos = T(1:3,4);
    
    j_twist = jacob0(S,q);
    
    J_w = [];
    J_v = [];
    
    for i = 1:s 
        J_w(:,i) = j_twist(1:3,i);
        J_v(:,i) = j_twist(4:6,i);
    end
    
    J_a = J_v - (skew(pos)*J_w);
    
end