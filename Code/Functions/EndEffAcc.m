% This function calculate the symbolic expression of linear acceleration
% given angular position, velocity and acceleration
function pdotdot = EndEffAcc(TE0,res,dres,ddres)
syms q1(t) q2(t) q3(t) q4(t) q5(t) q6(t)
syms dq1(t) dq2(t) dq3(t) dq4(t) dq5(t) dq6(t)
syms ddq1(t) ddq2(t) ddq3(t) ddq4(t) ddq5(t) ddq6(t)

p = TE0(1:3,end);
dp = subs(diff(p,t),{diff(q1(t),t),diff(q2(t),t),diff(q3(t),t),diff(q4(t),t),diff(q5(t),t),diff(q6(t),t)},...
             {dq1(t),dq2(t),dq3(t),dq4(t),dq5(t),dq6(t)});
         
ddp = subs(diff(dp,t),{diff(q1(t),t),diff(q2(t),t),diff(q3(t),t),diff(q4(t),t),diff(q5(t),t),diff(q6(t),t),...
                diff(dq1(t),t),diff(dq2(t),t),diff(dq3(t),t),diff(dq4(t),t),diff(dq5(t),t),diff(dq6(t),t)},...
                {dq1(t),dq2(t),dq3(t),dq4(t),dq5(t),dq6(t),ddq1(t),ddq2(t),ddq3(t),ddq4(t),ddq5(t),ddq6(t)});
            
for i = 1:length(res)
    pdotdot(:,i) = double(subs(ddp,{q1(t),q2(t),q3(t),q4(t),q5(t),q6(t),dq1(t),dq2(t),dq3(t),dq4(t),dq5(t),dq6(t),ddq1(t),ddq2(t),ddq3(t),ddq4(t),ddq5(t),ddq6(t)},...
                                   {res(i,1),res(i,2),res(i,3),res(i,4),res(i,5),res(i,6),dres(i,1),dres(i,2),dres(i,3),dres(i,4),dres(i,5),dres(i,6),...
                                   ddres(i,1),ddres(i,2),ddres(i,3),ddres(i,4),ddres(i,5),ddres(i,6)}));
end
end