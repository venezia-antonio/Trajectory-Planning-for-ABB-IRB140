% This function calculate the inverse kinematics for the whole manipulator,
% including each solution
function [s1,s2,s3,s4,s5,s6,s7,s8] = getIK(Tik,T30)
    [q11,q12] = get_q1(Tik);
    [q31,q32] = get_q3(q11,Tik);
    [q21,~,~,~] = get_q2(q11,q31,q32,Tik);
    [q41,q42,q51,q52,q61,q62] = get_Orientation(Tik,T30,q11,q21,q31);
    s1 = double([q11,q21,q31,q41,q51,q61]);

    [q31,q32] = get_q3(q11,Tik);
    [~,~,q23,~] = get_q2(q11,q31,q32,Tik);
    [q41,q42,q51,q52,q61,q62] = get_Orientation(Tik,T30,q11,q23,q32);
    s2 = double([q11,q23,q32,q41,q51,q61]);

    [q31,q32] = get_q3(q12,Tik);
    [~,q22,~,~] = get_q2(q12,q31,q32,Tik);
    [q41,q42,q51,q52,q61,q62] = get_Orientation(Tik,T30,q12,q22,q31);
    s3 = double([q12,q22,q31,q41,q51,q61]);

    [q31,q32] = get_q3(q12,Tik);
    [~,~,~,q24] = get_q2(q12,q31,q32,Tik);
    [q41,q42,q51,q52,q61,q62] = get_Orientation(Tik,T30,q12,q24,q32);
    s4 = double([q12,q24,q32,q41,q51,q61]);

    [q31,q32] = get_q3(q12,Tik);
    [~,q22,~,~] = get_q2(q12,q31,q32,Tik);
    [q41,q42,q51,q52,q61,q62] = get_Orientation(Tik,T30,q12,q22,q31);
    s5 = double([q12,q22,q31,q41,q51,q61]);

    [q31,q32] = get_q3(q12,Tik);
    [~,~,~,q24] = get_q2(q12,q31,q32,Tik);
    [q41,q42,q51,q52,q61,q62] = get_Orientation(Tik,T30,q12,q24,q32);
    s6 = double([q12,q24,q32,q41,q51,q61]);

    [q31,q32] = get_q3(q12,Tik);
    [~,q21,~,~] = get_q2(q12,q31,q32,Tik);
    [q41,q42,q51,q52,q61,q62] = get_Orientation(Tik,T30,q12,q21,q31);
    s7 = double([q12,q21,q31,q41,q51,q61]);

    [q31,q32] = get_q3(q11,Tik);
    [~,~,q23,~] = get_q2(q11,q31,q32,Tik);
    [q41,q42,q51,q52,q61,q62] = get_Orientation(Tik,T30,q11,q23,q32);
    s8 = double([q11,q23,q32,q41,q51,q61]);
end
