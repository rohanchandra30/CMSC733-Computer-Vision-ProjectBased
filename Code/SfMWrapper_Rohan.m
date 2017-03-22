%% Good Luck, Have fun!
clc
clear all

Nimages = 6;

%  Taken from calibration.txt %%
K = [568.996140852 0 643.21055941; 0 568.988362396 477.982801038; 0 0 1];

% Load the images and extract matching%%
points = cell(6,6);

matches = get_matches(Nimages);

[x1,x2] = get_points(matches{1}{2});
points{1,2} = [x1, x2];
[x1,x3] = get_points(matches{1}{3});
points{1,3} = [x1, x3];
[x1,x4] = get_points(matches{1}{4});
points{1,4} = [x1, x4];
[x2,x3] = get_points(matches{2}{3});
points{2,3} = [x2, x3];
[x2,x4] = get_points(matches{2}{4});
points{2,4} = [x2, x4];
[x3,x4] = get_points(matches{3}{4});
points{3,4} = [x3, x4];
[x3,x5] = get_points(matches{3}{5});
points{3,5} = [x3, x5];
[x3,x6] = get_points(matches{3}{6});
points{3,6} = [x3, x6];
[x4,x5] = get_points(matches{4}{5});
points{4,5} = [x4, x5];
[x4,x6] = get_points(matches{4}{6});
points{4,6} = [x4, x6];
[x5,x6] = get_points(matches{5}{6});
points{5,6} = [x5, x6];

for i=1:2
    for j = 1:2
        if ~isempty(points{i,j})
            [a1,a2,id,f] = GetInliersRANSAC(points{i,j}(:,1:2),points{i,j}(:,3:4));
            points{i,j} = [a1,a2];
            
            idx{i,j} = id;
            F{i,j} = f;
        end
    end
end
%% % Part 5

E = EssentialMatrixFromFundamentalMatrix(F{1,2},K);

[Cset,Rset] = ExtractCameraPose(E);


%% % Part 6


x1 = points{1,2}(:,1:2);
x2 = points{1,2}(:,3:4);

for i = 1:4
    
    Xset{i} = LinearTriangulation(K, zeros(3,1), eye(3), Cset{i}, Rset{i}, x1, x2) ;
    
end

[C, R, X] = DisambiguateCameraPose(Cset, Rset, Xset);

X_opt = NonlinearTriangulation(K, zeros(3,1), eye(3), C, R, x1, x2, X);