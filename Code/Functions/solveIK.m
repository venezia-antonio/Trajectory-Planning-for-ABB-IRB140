% This function calculate the inverse kinematics for each time instant
% considered.
% Change the output in order to use another solution of IK for ABB IRB 140
function set2 = solveIK(p_t,R_t,T30)
    h = waitbar(0,'Solving Inverse Kinematics, please wait...');
    for i = 1:length(p_t)
        p = [p_t(1,i) p_t(2,i) p_t(3,i)]';
        Tik = [R_t(i).R p;0 0 0 1];
        [s1,s2,s3,s4,s5,s6,s7,s8] = getIK(Tik,T30);
        set1(i,:) = s1;
        set2(i,:) = s2;
        set3(i,:) = s3;
        set4(i,:) = s4;
        set5(i,:) = s5;
        set6(i,:) = s6;
        set7(i,:) = s7;
        set8(i,:) = s8;
        waitbar(i/length(p_t),h)
    end
    close(h);
end