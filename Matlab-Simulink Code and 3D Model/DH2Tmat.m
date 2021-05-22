function [ Tmats ] = DH2Tmat( DHparam )
%DH2Tmat take DH parameters of a manipulator and returns Transformation matrics for each frame in manipulator (i.e. for each link and end effector excluding base)
%DHparam is n*4 matrix where n is no. of links or frames.Tmats is 4*4*n
%matrix which contains transformation matrix of each frame from base frame
%thus total of n matices.

 n = size(DHparam,1);   %no of frames (including end effector)
 Tmat = eye(4);
 Tmats = zeros(4,4,n);
 for i = 1:n
     
     cal = cos(deg2rad(DHparam(i,4)));
     sal = sin(deg2rad(DHparam(i,4)));
     cq =  cos(deg2rad(DHparam(i,1)));
     sq =  sin(deg2rad(DHparam(i,1)));
%      cal = cos(degtorad(DHparam(i,4)));
%      sal = sin(degtorad(DHparam(i,4)));
%      cq =  cos(degtorad(DHparam(i,1)));
%      sq = sin(degtorad(DHparam(i,1)));
     a = DHparam(i,3);
     d = DHparam(i,2);
     A = [cq -sq*cal sq*sal a*cq;sq cq*cal -cq*sal a*sq;0 sal cal d;0 0 0 1];
     Tmat = Tmat*A;
     Tmats(:,:,i) = Tmat;
 end
end

