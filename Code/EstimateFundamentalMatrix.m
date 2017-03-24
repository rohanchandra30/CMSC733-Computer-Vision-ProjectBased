
function F = EstimateFundamentalMatrix(xy,xy2)
xy = [xy ones(size(xy,1),1)];
xy2 = [xy2 ones(size(xy2,1),1)];
M = kron(xy2,[1 1 1]).*repmat(xy,1,3);
[~,~,V]=svd(M);
F=reshape(V(:,end),3,3)';
[FU,FD,FV]=svd(F);
FD(3,3) = 0;
F = FU*FD*FV';
end