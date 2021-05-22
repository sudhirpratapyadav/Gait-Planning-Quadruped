function [ positions ] = forwardKinematics( angles )
%forwardKinematics take angles of each leg and return positions of each
%point required total of 4*3 points (legs*no of frames per leg) and each
%point require 3 co-ordinates (x,y,z) thus it is a 3d matrix.

%angles variable is 4*3 matrix (legs*angles for each leg) (3 angles per leg as 3DOF leg is taken)
%positions variable is n+1*3*4 matrix n+1 as base (0 0 0) is also included
% 3 for each co-ordinate (x,y,z) and 4 is no of legs.

global dhparam;
dh1 = [angles(1,:)',dhparam];
dh2 = [angles(2,:)',dhparam];
dh3 = [angles(3,:)',dhparam];
dh4 = [angles(4,:)',dhparam];
Tmat1 = DH2Tmat(dh1);
Tmat2 = DH2Tmat(dh2);
Tmat3 = DH2Tmat(dh3);
Tmat4 = DH2Tmat(dh4);
positions = zeros(size(dhparam,1)+1,3,4);
for i = 1:size(dhparam,1)
    positions(i+1,:,1) = Tmat1(1:3,4,i)';
end
for i = 1:size(dhparam,1)
    positions(i+1,:,2) = Tmat2(1:3,4,i)';
end
for i = 1:size(dhparam,1)
    positions(i+1,:,3) = Tmat3(1:3,4,i)';
end
for i = 1:size(dhparam,1)
    positions(i+1,:,4) = Tmat4(1:3,4,i)';
end
end

