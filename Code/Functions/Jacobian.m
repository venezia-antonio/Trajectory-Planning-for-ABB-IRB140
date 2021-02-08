% This function calculate the geometric jacobian for the manipulator:
% -jointType R means that it has been considered revolute joint
% -jointType P means that it has been considered prismatic joint
% -T_Iminus1_0 is in a structure T
% -jointType is in a structure 
function J = Jacobian(T,T_E0,res)
    syms q1 q2 q3 q4 q5 q6
    J_I = [];
    for i = 1:length(T)
        if (T(i).JT == 'R')
            JP = cross(T(i).T_Iminus1_0(1:3,3),(T_E0(1:3,end)-T(i).T_Iminus1_0(1:3,end)));
            JO = T(i).T_Iminus1_0(1:3,3);
        else
            JP = T(i).T_Iminus1_0(1:3,3);
            JO = [0 0 0]';
        end
        J_I = [J_I,[JP;JO]];
        J = struct([]);
    end
        for i = 1:length(res)
            J(i).J = double(subs(J_I,{q1,q2,q3,q4,q5,q6},{res(i,1),res(i,2),res(i,3),res(i,4),res(i,5),res(i,6)}));
        end
end