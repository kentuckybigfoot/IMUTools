function [ angle x y z ] = convertQtoAA( Q )
%CONVERTQTOAA Convert quaternion data to axis angle.
%   Accept array of quaternion data arranged x y z and convert to axis
%   angle. Axis angle data output under variables angle, x, y, z.
%
%   http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/
%   used for reference.

angle = 2*acos(Q(1));

s = sqrt(1-Q(1)*Q(1));

if s < 0.001
    s = 1;
end

x = Q(2)/s;
y = Q(3)/s;
z = Q(4)/s;

end

