function [ angles ] = inverseKinematics( positions )

global linkLengths;
a1 = linkLengths(1);
a2 = linkLengths(2);
a3 = linkLengths(3);
x = positions(:,1);
y = positions(:,2);
z = positions(:,3);
L1 = sqrt(x.^2+y.^2);
L = sqrt(z.^2+(L1-a1).^2);
D = cos((pi-acos((a2^2+a3^2-L.^2)/(2*a2*a3))));

angles(:,1,1) = atan2(y,x);
angles(:,2,1) = atan2(z,(sqrt((L1-a1).^2)))-acos((L.^2+a2^2-a3^2)./(2*a2*L));
angles(:,3,1) = atan2(sqrt(1-D.^2),D);
angles(:,1,2) = atan2(y,x);
angles(:,2,2) = atan2(z,(sqrt((L1-a1).^2)))+acos((L.^2+a2^2-a3^2)./(2*a2*L));
angles(:,3,2) = atan2(-sqrt(1-D.^2),D);
angles = rad2deg(real(angles));
end

