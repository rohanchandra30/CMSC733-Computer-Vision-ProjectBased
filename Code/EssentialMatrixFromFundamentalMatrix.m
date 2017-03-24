function E = EssentialMatrixFromFundamentalMatrix(F,K)
%% Inputs:
% K: 3x3 camera intrinsic parameter
% F: fundamental matrix
%% Outputs:
% E: 3x3 essential matrix with singular values (1,1,0)

%% Your Code goes here

E = K'*F*K;

[U,D,V] = svd(E);
Corrected_Matrix = [1,0,0;0,1,0;0,0,0];

E = U*Corrected_Matrix*V';

E = E / norm(E);



end
