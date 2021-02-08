% This function calculates joint variable in the joint space using quintic
% polynomial
function [q1,q2,q3,q4,q5,q6,dq1,dq2,dq3,dq4,dq5,dq6,ddq1,ddq2,ddq3,ddq4,ddq5,ddq6] = trajPlan1(qi,qf,ti,tf,dqi,dqf,ddqi,ddqf,Ts)
    syms a0 a1 a2 a3 a4 a5 t

    for i = 1:length(qi)
        eq1 = subs(a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5,t,ti) == qi(i);
        eq2 = subs(a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5,t,tf) == qf(i);
        eq3 = subs(diff(a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5,t),t,ti) == dqi(i);
        eq4 = subs(diff(a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5,t),t,tf) == dqf(i);
        eq5 = subs(diff(a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5,t,2),t,ti) == ddqi(i);
        eq6 = subs(diff(a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5,t,2),t,tf) == ddqf(i);
        s = solve([eq1;eq2;eq3;eq4;eq5;eq6],[a0,a1,a2,a3,a4,a5]);
        b = double([s.a0,s.a1,s.a2,s.a3,s.a4,s.a5]);
        j = 1;
        for time = ti:Ts:tf
            [p,v,a] = poli5(b,time);
            eval(['q' num2str(i) '(j) = p;']);
            eval(['dq' num2str(i) '(j) = v;']);
            eval(['ddq' num2str(i) '(j) = a;']);
            j = j + 1;
        end
    end
end


