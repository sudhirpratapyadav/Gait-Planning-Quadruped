function [ positions ] = world2local( pos )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

global body2LegTmat;
global bodyTmat;
legTmat2body(:,:,1) = [[body2LegTmat(1:3,1:3,1)',-body2LegTmat(1:3,1:3,1)'*body2LegTmat(1:3,4,1)];0 0 0 1];
legTmat2body(:,:,2) = [[body2LegTmat(1:3,1:3,2)',-body2LegTmat(1:3,1:3,2)'*body2LegTmat(1:3,4,2)];0 0 0 1];
legTmat2body(:,:,3) = [[body2LegTmat(1:3,1:3,3)',-body2LegTmat(1:3,1:3,3)'*body2LegTmat(1:3,4,3)];0 0 0 1];
legTmat2body(:,:,4) = [[body2LegTmat(1:3,1:3,4)',-body2LegTmat(1:3,1:3,4)'*body2LegTmat(1:3,4,4)];0 0 0 1];
inv_bodyTmat = [[bodyTmat(1:3,1:3)',-bodyTmat(1:3,1:3)'*bodyTmat(1:3,4)];0 0 0 1];

p = (legTmat2body(:,:,1)*inv_bodyTmat*[pos(1,:)';1])';
positions(1,:) = p(:,1:end-1);
p = (legTmat2body(:,:,2)*inv_bodyTmat*[pos(2,:)';1])';
positions(2,:) = p(:,1:end-1);
p = (legTmat2body(:,:,3)*inv_bodyTmat*[pos(3,:)';1])';
positions(3,:) = p(:,1:end-1);
p = (legTmat2body(:,:,4)*inv_bodyTmat*[pos(4,:)';1])';
positions(4,:) = p(:,1:end-1);


end

