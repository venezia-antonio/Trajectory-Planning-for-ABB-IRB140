% This function calculates the rotation matrix given the axis and angle of
% rotation
function R = axisAngle(axis,angle)
    R = (angvec2r(angle,axis));
end