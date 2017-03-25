function [Cset,Rset] = ExtractCameraPose(E)
%% Input
% E: essential matrix
%% Output:
% Cset and Rset: four congurations of camera centers and rotations, 
% i.e., Cset{i}=Ci and Rset{i}=Ri.

%% Your Code goes here

% W matrix as described in PDF
W = [0,-1,0;1,0,0;0,0,1];

% Computing SVD of E for Calculation of C and R
[U, D, V] = svd(E);

C1 = U(:,3); R1 = U*W*V';
C2 = -U(:,3); R2 = U*W*V';
C3 = U(:,3); R3 = U*W'*V';
C4 = -U(:,3); R4 = U*W'*V';

[Cset, Rset] = Do_Correction(C1, C2, C3, C4, R1, R2, R3, R4);







end
