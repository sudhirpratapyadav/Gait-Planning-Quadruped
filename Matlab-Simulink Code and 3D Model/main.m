clear;
clc;

global linkLengths;     %Lengths of links (from body to tip)
global body2LegTmat;    %From body center to legs (4*4*4)
global dhparam;         %dh parameters of sigle leg
global bodyDim;         %Length and width of body
global bodyTmat;        %From ground to center of body transformation matrix


bodyDim = [0.1895 0.1895];
linkLengths = [0.048;0.083;0.095];
dhparam = [[0;0;0],linkLengths,[90;0;0]];%theta D A alpha


body2LegTmat = zeros(4,4,4);
body2LegTmat(:,:,1) = [-1 0 0 -bodyDim(2)/2;0 1 0 bodyDim(1)/2;0 0 1 0;0 0 0 1];
body2LegTmat(:,:,2) = [1 0 0 bodyDim(2)/2;0 1 0 bodyDim(1)/2;0 0 1 0;0 0 0 1];
body2LegTmat(:,:,3) = [-1 0 0 -bodyDim(2)/2;0 1 0 -bodyDim(1)/2;0 0 1 0;0 0 0 1];
body2LegTmat(:,:,4) = [1 0 0 bodyDim(2)/2;0 1 0 -bodyDim(1)/2;0 0 1 0;0 0 0 1];



gaits = [   0 0.5 0.75 0.25 0.75;... %crawl
            0 0.5 0.75 0.25 0.8;... %crawl
            0 0.5 0.5 0 0.5;...      %trot
            0 0.5 0 0.5 0.5;...      %pace   
            0 0 0.75 0.75 0.5];      %gallop
            %(phi1 phi2 phi3 phi4)pashes of legs then beta(dutyFactor) (1 4 2 3)
            
gait = 1;
% 1 for crawl
% 2 for trot
% 3 for pace
% 4 for gallop


cT = 4; %Cycle Time
Fc = 0.07; %foot clearance or foot hieght to be lifted
Hb = 0.09; %hieght of body;
sL = 0.14; %step length
bodyTmat = eye(4);
bodyTmat(3,4) = Hb;

%sL = analyseDesign();
vel = sL/(gaits(gait,5)*cT);
display(vel);


 
no_steps = 100;
trajectory = getLegTrajactory(gaits(gait,:),Fc,sL,Hb,no_steps);
% % w_footpos = [ -0.2148    0.1615   -0.0000;...
% %                 0.2148    0.0736         0;...
% %                 -0.2148   -0.0719         0;...
% %                 0.2148   -0.1530    0.0011];
% % 
% % figure
% % data = zeros(no_steps,4);
% % for i = 1:no_steps
% %     data(i,1) = (i*cT)/no_steps;
% %     data(i,2) = trajectory(1,1,i);
% %     data(i,3) = trajectory(1,2,i);
% %     data(i,4) = trajectory(1,3,i);
% %     
% %     data(i,5) = trajectory(2,1,i);
% %     data(i,6) = trajectory(2,2,i);
% %     data(i,7) = trajectory(2,3,i);
% %     
% %     data(i,8) = trajectory(3,1,i);
% %     data(i,9) = trajectory(3,2,i);
% %     data(i,10) = trajectory(3,3,i);
% %     
% %     data(i,11) = trajectory(4,1,i);
% %     data(i,12) = trajectory(4,2,i);
% %     data(i,13) = trajectory(4,3,i);
% % end
% % figure
% % subplot(2,2,1);
% % plot(data(:,1),data(:,2));
% % ylabel('X(cm)');
% % xlabel('Time(s)');
% % title('Trajectory of leg1 (X axis)');
% % 
% % subplot(2,2,2);
% % plot(data(:,1),data(:,3));
% % ylabel('Y(cm)');
% % xlabel('Time(s)');
% % title('Trajectory of leg1 (Y axis)');
% % 
% % subplot(2,2,3);
% % plot(data(:,1),data(:,4));
% % ylabel('Z(cm)');
% % xlabel('Time(s)');
% % title('Trajectory of leg1 (Z axis)');
% % 
% % subplot(2,2,4);
% % plot(data(:,3),data(:,4));
% % ylabel('Z(cm)');
% % xlabel('Y(cm)');
% % ylim([-0.1 -0.01]);
% % title('Trajectory of leg1 (Z vs Y)');
% % 
% % figure
% % plot(data(:,1),[data(:,4),data(:,7),data(:,10),data(:,13)]);
% % legend('leg1','leg2','leg3','leg4');
% % ylabel('Z(cm)');
% % xlabel('Time(s)');
% % title('Trajectory of different legs (Z axis)');
% % 
% 
% 
% % 
% % configs = zeros(4,3,4);
% % n = 1;
% % theta =  1;
% % rotX = [1 0 0 ;0 cos(deg2rad(theta)) -sin(deg2rad(theta));0 sin(deg2rad(theta)) cos(deg2rad(theta))];
% % rotY = [cos(deg2rad(theta)) 0 sin(deg2rad(theta)) ;0 1 0;-sin(deg2rad(theta)) 0 cos(deg2rad(theta))];
% % rotZ = [cos(deg2rad(theta)) -sin(deg2rad(theta)) 0;sin(deg2rad(theta)) cos(deg2rad(theta)) 0; 0 0 1];
% % 
% % 
% % for i = 1:5
% % bodyTmat = [rotY*bodyTmat(1:3,1:3),bodyTmat(1:3,4);0 0 0 1]; 
% % l_footpos = world2local(w_footpos);
% % leg_angles = inverseKinematics(l_footpos);
% % positions = forwardKinematics(leg_angles(:,:,2));
% % world_pos = local2world(positions);
% % configs(:,:,:,n) = world_pos;
% % n =n+1;
% % end
% % 
% % for i = 1:5
% % bodyTmat = [rotX*bodyTmat(1:3,1:3),bodyTmat(1:3,4);0 0 0 1]; 
% % l_footpos = world2local(w_footpos);
% % leg_angles = inverseKinematics(l_footpos);
% % positions = forwardKinematics(leg_angles(:,:,2));
% % world_pos = local2world(positions);
% % configs(:,:,:,n) = world_pos;
% % n =n+1;
% % end
% % 
% % 
% % for i = 1:5
% % bodyTmat = [rotX'*bodyTmat(1:3,1:3),bodyTmat(1:3,4);0 0 0 1]; 
% % l_footpos = world2local(w_footpos);
% % leg_angles = inverseKinematics(l_footpos);
% % positions = forwardKinematics(leg_angles(:,:,2));
% % world_pos = local2world(positions);
% % configs(:,:,:,n) = world_pos;
% % n =n+1;
% % end
% % 
% % for i = 1:5
% % bodyTmat = [rotY'*bodyTmat(1:3,1:3),bodyTmat(1:3,4);0 0 0 1]; 
% % l_footpos = world2local(w_footpos);
% % leg_angles = inverseKinematics(l_footpos);
% % positions = forwardKinematics(leg_angles(:,:,2));
% % world_pos = local2world(positions);
% % configs(:,:,:,n) = world_pos;
% % n =n+1;
% % end
% % 
% % for i = 1:5
% % bodyTmat = [rotZ'*bodyTmat(1:3,1:3),bodyTmat(1:3,4);0 0 0 1]; 
% % l_footpos = world2local(w_footpos);
% % leg_angles = inverseKinematics(l_footpos);
% % positions = forwardKinematics(leg_angles(:,:,2));
% % world_pos = local2world(positions);
% % configs(:,:,:,n) = world_pos;
% % n =n+1;
% % end
% % 
% % for i = 1:5
% % bodyTmat = [rotZ*bodyTmat(1:3,1:3),bodyTmat(1:3,4);0 0 0 1]; 
% % l_footpos = world2local(w_footpos);
% % leg_angles = inverseKinematics(l_footpos);
% % positions = forwardKinematics(leg_angles(:,:,2));
% % world_pos = local2world(positions);
% % configs(:,:,:,n) = world_pos;
% % n =n+1;
% % end
% % 
% % n = n-1;
% % figure
% % for i = 1:n
% %     draw(configs(:,:,:,i));
% %     pause(0.1);    
% % end
% % 
% % 
% % 
% bodyTmat = eye(4);
% bodyTmat(3,4) = Hb;
% % 
figure
configs = zeros(4,3,4);
n = 1;

singleLegAngles = zeros(no_steps,3);

motor1 = zeros(no_steps,2);
motor2 = zeros(no_steps,2);
motor3 = zeros(no_steps,2);
motor4 = zeros(no_steps,2);
motor5 = zeros(no_steps,2);
motor6 = zeros(no_steps,2);
motor7 = zeros(no_steps,2);
motor8 = zeros(no_steps,2);
motor9 = zeros(no_steps,2);
motor10 = zeros(no_steps,2);
motor11 = zeros(no_steps,2);
motor12 = zeros(no_steps,2);
for i=1:no_steps
    ind = mod(i,no_steps);
    if(ind ==0)
        ind = no_steps;
    end
    l_footpos = [   0.15 trajectory(1,2,ind) trajectory(1,3,ind);...
                    0.15 trajectory(2,2,ind) trajectory(2,3,ind);...
                    0.15 trajectory(3,2,ind) trajectory(3,3,ind);...
                    0.15 trajectory(4,2,ind) trajectory(4,3,ind)];
   bodyTmat(2,4) = (i*sL)/(gaits(gait,5)*no_steps);
   leg_angles = inverseKinematics(l_footpos);
%   display(leg_angles);
    if(i<=no_steps)
        singleLegAngles(i,:,2) = leg_angles(1,:,2);
        motor1(i,1) = (i*cT)/no_steps;
        motor2(i,1) = (i*cT)/no_steps;
        motor3(i,1) = (i*cT)/no_steps;
        motor4(i,1) = (i*cT)/no_steps;
        motor5(i,1) = (i*cT)/no_steps;
        motor6(i,1) = (i*cT)/no_steps;
        motor7(i,1) = (i*cT)/no_steps;
        motor8(i,1) = (i*cT)/no_steps;
        motor9(i,1) = (i*cT)/no_steps;
        motor10(i,1) = (i*cT)/no_steps;
        motor11(i,1) = (i*cT)/no_steps;
        motor12(i,1) = (i*cT)/no_steps;
        motor1(i,2) = leg_angles(1,1,2);
        motor2(i,2) = leg_angles(1,2,2);
        motor3(i,2) = leg_angles(1,3,2);
        motor4(i,2) = leg_angles(2,1,2);
        motor5(i,2) = leg_angles(2,2,2);
        motor6(i,2) = leg_angles(2,3,2);
        motor7(i,2) = leg_angles(3,1,2);
        motor8(i,2) = leg_angles(3,2,2);
        motor9(i,2) = leg_angles(3,3,2);
        motor10(i,2) = leg_angles(4,1,2);
        motor11(i,2) = leg_angles(4,2,2);
        motor12(i,2) = leg_angles(4,3,2);
    end
   positions = forwardKinematics(leg_angles(:,:,2));
   world_pos = local2world(positions);
   configs(:,:,:,n) = world_pos;
   n =n+1;
end
% 
% motor1(:,2) = motor1(:,2)-motor1(1,2);
% motor2(:,2) = motor2(:,2)-motor2(1,2);
% motor3(:,2) = motor3(:,2)-motor3(1,2);
% motor4(:,2) = motor4(:,2)-motor4(1,2);
% motor5(:,2) = motor5(:,2)-motor5(1,2);
% motor6(:,2) = motor6(:,2)-motor6(1,2);
% motor7(:,2) = motor7(:,2)-motor7(1,2);
% motor8(:,2) = motor8(:,2)-motor8(1,2);
% motor9(:,2) = motor9(:,2)-motor9(1,2);
% motor10(:,2) = motor10(:,2)-motor10(1,2);
% motor11(:,2) = motor11(:,2)-motor11(1,2);
% motor12(:,2) = motor12(:,2)-motor12(1,2);
% 
% csvwrite('motor1.csv',motor1);
% csvwrite('motor2.csv',motor2);
% csvwrite('motor3.csv',motor3);
% csvwrite('motor4.csv',motor4);
% csvwrite('motor5.csv',motor5);
% csvwrite('motor6.csv',motor6);
% csvwrite('motor7.csv',motor7);
% csvwrite('motor8.csv',motor8);
% csvwrite('motor9.csv',motor9);
% csvwrite('motor10.csv',motor10);
% csvwrite('motor11.csv',motor11);
% csvwrite('motor12.csv',motor12);
% display([motor1(:,2),motor2(:,2),motor3(:,2),motor4(:,2),motor5(:,2),motor6(:,2),motor7(:,2),motor8(:,2),motor9(:,2),motor10(:,2),motor11(:,2),motor12(:,2)]);
% display([motor1(1,2),motor2(1,2),motor3(1,2);motor4(1,2),motor5(1,2),motor6(1,2);motor7(1,2),motor8(1,2),motor9(1,2);motor10(1,2),motor11(1,2),motor12(1,2)]);


n = n-1;

stm = zeros(n,1);

fig1 = figure;
M(n) = struct('cdata',[],'colormap',[]);
for i = 1:no_steps
    stm(i) = draw(configs(:,:,:,i));
    pause(0.01);
   % M(i) = getframe(fig1);
end




% 
%     
% fig1 =  figure;
% for i = 1:no_steps
%     plot(linspace(0,(i*cT)/no_steps,i),stm(1:i));
%     hold on
%     plot([0 12],[0 0]);
%     hold off
%     title('Static Stability Limit in Y Crawl');
%     ylabel('Static Stablity Limit (cm)');
%     xlabel('Time (s)');
%     xlim([0 4]);
%     ylim([-1 12]);
%     drawnow;
%     pause(0.05);
% end
% myVideo = VideoWriter('myfile.avi');
% uncompressedVideo = VideoWriter('myfile.avi', 'Uncompressed AVI');
% myVideo.FrameRate = no_steps/cT;  % Default 30
% myVideo.Quality = 100;    % Default 75
% open(myVideo);
% writeVideo(myVideo, M);
% close(myVideo);

angular_vel = ((no_steps*diff(singleLegAngles(:,:,2)))/cT)/6;
acc = diff(angular_vel)*6;


figure
plot(linspace(0,size(singleLegAngles(:,:,2),1)*cT/no_steps,size(singleLegAngles(:,:,2),1)),singleLegAngles(:,:,2));
legend('angle 1','angle 2','angle 3');
title('Single Leg (leg 1) Joint angles');
ylabel('Angles (degree)');
xlabel('Time (s)');

figure
plot(linspace(0,size(angular_vel,1)*cT/no_steps,size(angular_vel,1)),angular_vel);
legend('omega 1','omega 2','omega 3');
display(max(angular_vel));
title('Single Leg (leg 1) Joint velocities ');
ylabel('Joint Angular Velocity (rpm)');
xlabel('Time (s)');

figure
plot(linspace(0,size(angular_vel,1)*cT/no_steps,size(acc,1)),deg2rad(acc));
legend('alpha 1','alpha 2','alpha 3');
display(max(angular_vel));
title('Single Leg (leg 1) Joint acceleration ');
ylabel('Joint Angular Acceleration (radian/(s^2))');
xlabel('Time (s)');


    
