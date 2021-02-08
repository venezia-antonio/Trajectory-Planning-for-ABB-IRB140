% This function calculate the inverse kinematics for Joint 3
function [q31,q32] = get_q3(q1,Tik)
    global L0 L1 L2 L3 L4 Le
    W = Tik(1:3,4) - (L4+Le)*Tik(1:3,3);
    Wx = W(1);Wy = W(2); Wz = W(3);
    r_2 = (Wx - L0*cos(q1))^2 + (Wy - L0*sin(q1))^2;
    s_2 = (Wz - L1)^2;
    h_2 = r_2 + s_2;
    cosq3 = (h_2 - L2^2 - L3^2)/(2*L2*L3);
    sinq3 = [sqrt(1-cosq3^2) -sqrt(1-cosq3^2)];
    q31 = (atan2(sinq3(1),cosq3))+ sym(pi)/2;
    q32 = (atan2(sinq3(2),cosq3))+ sym(pi)/2;
end