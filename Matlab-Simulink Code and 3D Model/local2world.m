function [ positions ] = local2world( pos )
global body2LegTmat;
global bodyTmat;
p = (bodyTmat*body2LegTmat(:,:,1)*[pos(:,:,1)';ones(1,size(pos,1))])';
positions(:,:,1) = p(:,1:end-1);
p = (bodyTmat*body2LegTmat(:,:,2)*[pos(:,:,2)';ones(1,size(pos,1))])';
positions(:,:,2) = p(:,1:end-1);
p = (bodyTmat*body2LegTmat(:,:,3)*[pos(:,:,3)';ones(1,size(pos,1))])';
positions(:,:,3) = p(:,1:end-1);
p = (bodyTmat*body2LegTmat(:,:,4)*[pos(:,:,4)';ones(1,size(pos,1))])';
positions(:,:,4) = p(:,1:end-1);
end

