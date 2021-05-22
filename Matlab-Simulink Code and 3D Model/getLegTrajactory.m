function [ trajectory ] = getLegTrajactory( gait,Fc,sL,Hb,no_steps )

 
trajectory = zeros(4,3,no_steps);

tdata = zeros(no_steps,3);

A = -(120*sL)/(gait(5)*(gait(5)-1).^5);
B = (sL*(gait(5).^5-3*gait(5)^4+10*gait(5).^2+5*gait(5)-1))/(2*((gait(5)-1).^5));
for i = 1:no_steps*gait(5)
    tdata(i,3) = -Hb ;
    tdata(i,2) = sL/2-((i-1)*sL)/(no_steps*gait(5)-1);
end


for i = no_steps*gait(5)+1:no_steps
    
    tdata(i,3) = Fc*(1-cos( 2*pi*((i-no_steps*gait(5))-no_steps*gait(1))/(no_steps*(1-gait(5))) ))/2-Hb ;
%     tdata(i,2) = -sL/2 + ((i-no_steps*gait(5))*sL)/(no_steps*(1-gait(5))+1);
 %   tdata(i,2) = -(sL*cos(((i-no_steps*gait(5))*pi)/(no_steps*(1-gait(5)))))/2;
      t = i/no_steps;
      tdata(i,2) = A*((t^5)/20-((gait(5)+1)*t^4)/8 +((gait(5)^2+4*gait(5)+1)*t^3)/12 -(gait(5)*(gait(5)+1)*t^2)/4 + (gait(5)*gait(5)*t)/4) - (sL*t)/gait(5) + B;

end


trajectory(1,3,:) = circshift(tdata(:,3),gait(1)*no_steps);
trajectory(2,3,:) = circshift(tdata(:,3),gait(2)*no_steps);
trajectory(3,3,:) = circshift(tdata(:,3),gait(3)*no_steps);
trajectory(4,3,:) = circshift(tdata(:,3),gait(4)*no_steps);

trajectory(1,2,:) = circshift(tdata(:,2),gait(1)*no_steps);
trajectory(2,2,:) = circshift(tdata(:,2),gait(2)*no_steps);
trajectory(3,2,:) = circshift(tdata(:,2),gait(3)*no_steps);
trajectory(4,2,:) = circshift(tdata(:,2),gait(4)*no_steps);

end


