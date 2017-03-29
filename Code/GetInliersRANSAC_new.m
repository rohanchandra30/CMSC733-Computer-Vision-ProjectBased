function [y1,y2,index,F] = GetInliersRANSAC_new(x1,x2, ind_x, threshold)
%% Input:
% x1 and x2: Nx2 matrices whose row represents a correspondence.
%% Output:
% y1 and y2: Nix2 matrices whose row represents an inlier correspondence 
%            where Ni is the number of inliers.
% idx: Nx1 vector that indicates ID of inlier y1.

F = [];
%% Your Code goes here
% x1 = cell2mat(x1);
% x2 = cell2mat(x2);

n=0;  

pts1 = zeros(8,2);
pts2 = zeros(8,2);

for i=1:1000 
  msize = size(ind_x,1);  
  idx = randperm(msize);
% Choose 8 correspondences, x?1 and x?2 randomly 
  for subset = 1:8
      pts1(subset,:) = cell2mat(x1(ind_x(idx(subset)),:));
      pts2(subset,:) = cell2mat(x2(ind_x(idx(subset)),:));
  end    
  
  f = EstimateFundamentalMatrix(pts1, pts2);
  
  S = [];
  for j=1:msize
    if constraint(cell2mat(x2(ind_x(j),:)), f, cell2mat(x1(ind_x(j),:))) < threshold
        S = [S; ind_x(j)];
    end
  end
  
  if n < size(S,1) 
    n = size(S,1);
    index = S; 
    F = f;
  end
end
y1 = cell2mat(x1(index,:));
y2 = cell2mat(x2(index,:));
end

function f = constraint(x2, F, x1)
   X2 = [ x2 1];
   X1 = [ x1 1];
   f = abs(X2*F*X1');
end
