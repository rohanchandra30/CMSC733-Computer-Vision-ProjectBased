function [] = plotfunc(A)
close all;
B = A(:,3);
idx_to_keep = B>0; 
D1 = A(find(idx_to_keep),1);
D2 = A(find(idx_to_keep),2);
D3 = A(find(idx_to_keep),3);
D = [D1, D2, D3];
plot3(D(:,1), D(:,2), D(:,3),'b.')
xlim([-50 50])
ylim([-50 50])
zlim([-50 50])

end
