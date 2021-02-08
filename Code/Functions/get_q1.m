% This function calculate the inverse kinematics for Joint 1
function [q11,q12] = get_q1(Tik)
% L4=0.046;Le = 0.16;
global L0 L1 L2 L3 L4 Le
W = Tik(1:3,4) - (L4+Le)*Tik(1:3,3);
Wx = W(1);Wy = W(2); Wz = W(3);
q11 = atan2(Wy,Wx);
if(Wy>=0)
    q12 = atan2(Wy,Wx) - sym(pi);
else
    q12 = atan2(Wy,Wx) + sym(pi);
end
end
