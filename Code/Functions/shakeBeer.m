% This function calculates the rotation matrices needed to change the 
% orientation of end-effector during the shake of the beer
% Notice that, after having determined the end-effector frame during the
% shake, a post-multiplication of rotation matrix R is mandatory in order
% to obtain a feasible orientation for the end-effector (could not be
% necessary).
function T = shakeBeer(cfr,c,ro,alpha,R)
    h = double(tan(alpha)*ro);      
    vertice = [c(1);c(2);c(3)+h];   

    for i = 1:length(cfr)
        versore(:,i) = (vertice - cfr(:,i))./norm(vertice - cfr(:,i));
        assi(:,i) = versore(:,i);
    end
    
    Xver = [1 0 0]';        % X axis versor
    Yver = [0 1 0]';        % Y axis versor
    Zver = [0 0 1]';        % Z axis versor
    
    cos_dir = zeros(3,length(cfr));
    for i = 1:length(cfr)
        cos_dir(1,i) = dot(assi(:,i),Xver);
        cos_dir(2,i) = dot(assi(:,i),Yver);
        cos_dir(3,i) = dot(assi(:,i),Zver);
    end
    
    axis_x = zeros(3,length(cfr));
    axis_y = zeros(3,length(cfr));
    axis_z = zeros(3,length(cfr));
    
    for i = 1:length(cfr)
    u = cos_dir(:,i); 
    v = [-cos_dir(3,i),0,cos_dir(1,i)]';
    w = cross(v,u);
    u = u/norm(u);
    v = v/norm(v);
    w = w/norm(w);
    axis_x(:,i) = v; 
    axis_y(:,i) = u;
    axis_z(:,i) = w;
    end
    
    T = struct([]);
    for i = 1:length(axis_x)
        T(i).R = [axis_x(:,i) axis_y(:,i) axis_z(:,i)]*R;
    end 
end