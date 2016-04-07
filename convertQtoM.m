function [ M ] = convertQtoM( Q )
%CONVERTQTOM Convert quaternion data to rotation matrix.
%   Input is array of quaternion data arranged [w,x,y,z]. Output is a
%   horizontal matrix with col 1:3 corresponding to a matrix of 1,1:3, col
%   4-6 corresponding to a matrix of 2,1:3, and so on.

qw = Q(1);
qx = Q(2);
qy = Q(3);
qz = Q(4);

R11 = (1 - 2*qy^2 - 2*qz^2);
R12 = (2*qx*qy - 2*qz*qw);
R13 = (2*qx*qz + 2*qy*qw);
R21 = (2*qx*qy + 2*qz*qw);
R22 = (1 - 2*qx^2 - 2*qz^2);
R23 = (2*qy*qz - 2*qx*qw);
R31 = (2*qx*qz - 2*qy*qw);
R32 = (2*qy*qz + 2*qx*qw);
R33 = (1 - 2*qx^2 - 2*qy^2);

M = [R11 R12 R13 R21 R22 R23 R31 R32 R33];

end

