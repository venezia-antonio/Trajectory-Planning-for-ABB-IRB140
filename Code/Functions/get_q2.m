% This function calculate the inverse kinematics for Joint 2
function [q21,q22,q23,q24] = get_q2(q1,q31,q32,Tik)
    global L0 L1 L2 L3 L4 Le
    W = Tik(1:3,4) - (L4+Le)*Tik(1:3,3);
    Wx = W(1);Wy = W(2); Wz = W(3);
    r(1) = sqrt((Wx - L0*cos(q1))^2 + (Wy - L0*sin(q1))^2);
    r(2) = -sqrt((Wx - L0*cos(q1))^2 + (Wy - L0*sin(q1))^2);
    s = (Wz - L1);
    psi = (q31 - sym(pi)/2);
    q21 = atan2(s,r(1)) - atan2(L3*sin(psi),L3*cos(psi)+L2);
    q22 = atan2(s,r(2)) - atan2(L3*sin(psi),L3*cos(psi)+L2);
    psi1 = (q32 - sym(pi)/2);
    q23 = atan2(s,r(1)) - atan2(L3*sin(psi1),L3*cos(psi1)+L2);
    q24 = atan2(s,r(2)) - atan2(L3*sin(psi1),L3*cos(psi1)+L2);
end