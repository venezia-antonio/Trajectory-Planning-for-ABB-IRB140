%% This function allows to plot ABB IRB 140 workspace
% Input:
%       1) TE0 symbolic matrix
%       2) q, which is a cell containing the angle limits for the robot

function plotWorkspace(TE0,q)
syms q1(t) q2(t) q3(t) q4(t) q5(t) q6(t)
% Add to path function's folder
addpath('Functions');

% Create a vector of symbolic variables
var = sym('q',[6 1]);

% Delete time dipendency from symbolic variables within TE0
TE0 = subs(TE0,{q1(t),q2(t),q3(t),q4(t),q5(t),q6(t)},{var(1),var(2),var(3),var(4),var(5),var(6)});

% Generate a grid of joint variables
[Q{1:numel(q)}] = ndgrid(q{:}); 
T = simplify(vpa(TE0,3));
Pos = T(1:3,end);
x(var(:)) = Pos(1);
X = matlabFunction(x);
X = X(Q{:});
y(var(:)) = Pos(2);
Y = matlabFunction(y);
Y = Y(Q{:});
z(var(:)) = Pos(3);
Z = matlabFunction(z);
Z = Z(Q{:});

% Plot workspace
figure()
plot3(X(:),Y(:),Z(:),'.r')
xlabel('X')
ylabel('Y')
zlabel('Z')
title('ABB IRB-140 Workspace')
end