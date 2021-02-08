% This function calculate the circular path in terms of position, velocity, acceleration
% in the operational space
function [cfr,dcfr,ddcfr] = trajCircle(r,d,Pi,s,ds,dds,R)  
    delta = Pi-d;
    c = d + (delta'*r)*r;
    ro = norm(Pi-c);
    for i = 1:length(s)
      cfr(:,i) = R*[ro*cos(s(i)/ro); ro*sin(s(i)/ro);0] + c;
      dcfr(:,i) = R*[-ds(i)*sin(s(i)/ro); ds(i)*cos(s(i)/ro);0];
      ddcfr(:,i) = R*[-ds(i)^2*cos(s(i)/ro)/ro-dds(i)*sin(s(i)/ro); -ds(i)^2*sin(s(i)/ro)/ro+dds(i)*cos(s(i)/ro);0];
    end
end