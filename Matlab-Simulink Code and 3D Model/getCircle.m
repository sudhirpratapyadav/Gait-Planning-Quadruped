function [ locus ] = getCircle( x,y,r,delta_theta,initial_angle,final_angle )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
delta_theta = deg2rad(delta_theta);
initial_angle = deg2rad(initial_angle);
final_angle = deg2rad(final_angle);
n = (final_angle-initial_angle)/delta_theta;
locus = zeros(n+1,2);
for i=0:n
    locus(i+1,:) = [x+r*cos(initial_angle+i*delta_theta),y+r*sin(initial_angle+i*delta_theta)]; 
end
end

