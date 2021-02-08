% This function generate the curbilinear absissa with a quintic polynomial
function [s,ds,dds] = curv(si,sf,ti,tf,Ts)
    syms a0 a1 a2 a3 a4 a5 t
    eq1 = subs(a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5,t,ti) == si;
    eq2 = subs(a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5,t,tf) == sf;
    eq3 = subs(diff(a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5,t),t,ti) == 0;
    eq4 = subs(diff(a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5,t),t,tf) == 0;
    eq5 = subs(diff(a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5,t,2),t,ti) == 0;
    eq6 = subs(diff(a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5,t,2),t,tf) == 0;
    sol = solve([eq1;eq2;eq3;eq4;eq5;eq6],[a0,a1,a2,a3,a4,a5]);
    b = double([sol.a0,sol.a1,sol.a2,sol.a3,sol.a4,sol.a5]);
    j = 1;
    for time = ti:Ts:tf
        [p,v,a] = poli5(b,time);
        s(j) = p;
        ds(j) = v;
        dds(j) = a;
        j = j + 1;
    end
end