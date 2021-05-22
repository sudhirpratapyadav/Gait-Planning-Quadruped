function [sl] = analyseDesign()
%UNTITLED2 Summary of this function goes here
%Detailed explanation goes here
global body2LegTmat;    %From body center to legs (4*4*4)
global dhparam;         %dh parameters of sigle leg
global bodyDim;         %Length and width of body
global bodyTmat;        %From ground to center of body transformation matrix

angleLimits = [-90 180 -90 90 -135 -45];
steps = 40;
pos = zeros((steps+1)^3+1,3);
ind = 1;
for i=0:steps
    for j=0:steps
        for k=0:steps
                tmat = DH2Tmat([ [ angleLimits(1)+(i*(angleLimits(2)-angleLimits(1)))/steps;...
                                   angleLimits(3)+(j*(angleLimits(4)-angleLimits(3)))/steps;...
                                   angleLimits(5)+(k*(angleLimits(6)-angleLimits(5)))/steps]...
                                ,dhparam]);
                
                pos(ind,:) = tmat(1:3,4,end)';
                ind=ind+1;
        end
    end
end
pos(:,3) = round(pos(:,3),2);
pos = sortrows(pos,3);
pevind = 1;
plts = 1;
ax_limit = 0.2;
figure
for i=1:size(pos,1)-1
    if(pos(i,3)~=pos(i+1,3)) 
        subplot(4,10,plts);
        plts = plts+1;
        plot(pos(pevind:i,1),pos(pevind:i,2),'.')
        xlim([-ax_limit ax_limit]);
        ylim([-ax_limit ax_limit]);
        titl = strcat('z=',num2str(-pos(i,3)));
        title(titl);
        pevind = i+1;
    end
end
subplot(4,10,plts);
plot(pos(pevind:end,1),pos(pevind:end,2),'.')
xlim([-ax_limit ax_limit]);
ylim([-ax_limit ax_limit]);
titl = strcat('z=',num2str(-pos(end,3)));
title(titl);

% 
% subplot(6,10,plts+1);
% plot(-0.3,-.3,'.')
% xlim([-0.3 0.3]);
% ylim([-0.3 0.3]);
% xlabel('X');
% ylabel('Y');
% titl = strcat('showing axes');
% title(titl);
        
        
pos = pos(pos(:,3)==-0.09,1:2);
dis = sqrt(pos(:,1).^2+pos(:,2).^2);
r = max(dis);
cir = getCircle(0,0,r,1,0,270);
sl = r*sqrt(2);
display(sl);
figure
plot(pos(:,1),pos(:,2),'.','MarkerSize',10)
hold on
plot(cir(:,1),cir(:,2),'.k','MarkerSize',10);
rectangle('Position',[-r/sqrt(2),0,sl,sl/2],'LineWidth',2);
rectangle('Position',[-r/sqrt(2),-r/sqrt(2),sl/2,sl],'LineWidth',2);
plot(0,sl/4,'.r','MarkerSize',20);
plot([-sl/2,sl/2],[sl/4,sl/4],'-r','LineWidth',2);
plot(-sl/4,0,'.g','MarkerSize',20);
plot([-sl/4,-sl/4],[sl/2,-sl/2],'-g','LineWidth',2);
xlabel('X')
ylabel('Y')
hold off
axis equal
title('z = 0.09');



l_footpos = [sl/4 0 -0.09;sl/4 0 -0.09;sl/4 0 -0.09;sl/4 0 -0.09];
leg_angles = inverseKinematics(l_footpos);
display(leg_angles);
positions = forwardKinematics(leg_angles(:,:,2));
world_pos = local2world(positions);
figure
 hold on
draw(world_pos);
pos = [pos,-0.09*ones(size(pos,1),1)];
positions = zeros(size(pos,1),size(pos,2),4);
positions(:,:,1) = pos;
positions(:,:,2) = pos;
positions(:,:,3) = pos;
positions(:,:,4) = pos;
% pos = local2world(positions);
% plot3(pos(:,1,1),pos(:,2,1),pos(:,3,1),'.g','MarkerSize',0.5);
% plot3(pos(:,1,2),pos(:,2,2),pos(:,3,2),'.g','MarkerSize',0.5);
% plot3(pos(:,1,3),pos(:,2,3),pos(:,3,3),'.g','MarkerSize',0.5);
% plot3(pos(:,1,4),pos(:,2,4),pos(:,3,4),'.g','MarkerSize',0.5);
hold off
ylim([-0.3 0.3]);
xlim([-0.3 0.3]);

end

