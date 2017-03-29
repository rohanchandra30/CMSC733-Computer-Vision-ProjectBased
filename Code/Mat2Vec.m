function [A1] = Mat2Vec(A)
m = size(A,1);
n = size(A,2);
mn = m*n;
A1 = reshape(A,mn,1);
end