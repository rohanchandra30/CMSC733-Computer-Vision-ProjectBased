%% Good Luck, Have fun!
clc
clear all
close all

addpath sba/matlab/


Nimages = 6;
I1 = im2double(imread('../Data/1.jpg'));
I2 = im2double(imread('../Data/2.jpg'));
I3 = im2double(imread('../Data/3.jpg'));
I4 = im2double(imread('../Data/4.jpg'));
I5 = im2double(imread('../Data/5.jpg'));
I6 = im2double(imread('../Data/6.jpg'));
I{1} = I1;
I{2} = I2;
I{3} = I3;
I{4} = I4;
I{5} = I5;
I{6} = I6;
K = [568.996140852 0 643.21055941; 0 568.988362396 477.982801038; 0 0 1];

%% Part 4 Calculating point correspondences and Fundamental Matrix
%

if  ~exist('variables_new.mat','file')
    [points, fpoints] = get_point_cell(Nimages);
    %[Points,F] = get_pointsandF_after_RANSAC(points);
    [Points,indx, F] = get_pointsandF_after_RANSAC_modified(fpoints, Nimages);
    save('variables_new.mat');
else
    load variables_new.mat
    
end



% These are the points which we got after RANSAC. They cause the non-linear
% function to hang. This means there is a problem in how we are getting x1
% and x2
x1 = Points{1,2}(:,1:2);
x2 = Points{1,2}(:,3:4);

% These are the x1, x2 from the coursera pipeline. These work. When using
% these points, comment out line 16.
% x1 = data.x1;
% x2 = data.x2;

figure;
imshow(I1);hold on;
plot(x1(:,1),x1(:,2),'r.');
hold off;
figure;
imshow(I2);hold on;
plot(x2(:,1),x2(:,2),'r.');
hold off;
dispMatchedFeatures(I1,I2,x1,x2, 'montage');

%% % Part 5 Calculating Esssential Matrix and the 4 poses of the second Camera

F = EstimateFundamentalMatrix(x1, x2);
E = EssentialMatrixFromFundamentalMatrix(F,K);
[Cset,Rset] = ExtractCameraPose(E);


%% % Part 6 Triangulating the 3D points and optimizing them

for i = 1:4
    Xset{i} = LinearTriangulation(K, zeros(3,1), eye(3), Cset{i}, Rset{i}, x1, x2) ;
end

[C, R, X] = DisambiguateCameraPose(Cset, Rset, Xset);

[X_opt, id] = NonlinearTriangulation(K, zeros(3,1), eye(3), C, R, x1, x2, X);

X_opt = X_opt(id>0, :);
x1 = x1(id>0, :);
x2 = x2(id>0, :);
indx{1,2} = indx{1,2}(id>0,:);

[t1, t2] = testX(X, zeros(3, 1), eye(3), R, C, K);

% Display reprojection points
DisplayCorrespondence(I2, x2, t2);
saveas(gcf, '../Report/2view_repo_img2.jpg');
DisplayCorrespondence(I1, x1, t1);
saveas(gcf, '../Data/2view_repo_img1.jpg');

% Plot and save the top view
plotfunc(X_opt);
saveas(gcf, '../Data/2view_topview.fig')

Cset = cell(Nimages, 1);
Rset = cell(Nimages, 1);
Xset = cell(Nimages, 1);
Rset{1,1} = eye(3);
Cset{1,1} = zeros(3,1);
Rset{2,1} = R;
Cset{2,1} = C;
Xset{1} = [];
Xset{2} = X_opt;
cP{1} = K*Rset{1,1}*[eye(3) -Cset{1,1}];
cP{2} = K*Rset{2,1}*[eye(3) -Cset{2,1}];
X = X_opt;
%fpoints = fpoints(2:end,:);
idx =indx{1,2};
%m = zeros(size(fpoints,1), size(fpoints,2),2);
measure(1:2*length(idx),Nimages) = 0;
t = 1:2:2*length(idx);
c = cell2mat(fpoints(idx,1));
measure(1:2:2*length(idx),1) = c(:,1);
measure(2:2:2*length(idx),1) = c(:,2);
c = cell2mat(fpoints(idx,2));
measure(1:2:2*length(idx),2) = c(:,1);
measure(2:2:2*length(idx),2) = c(:,2);
len = 2*length(idx);

for i = 3:Nimages
    
    id = intersect(indx{i-2, i-1}, find(cellfun(@(x) ~isempty(x), fpoints(:,i))));
    x = cell2mat(fpoints(id,i));
    X_n = [];
    for m = 1:length(id)
      t = find(indx{i-2, i-1} == id(m));
%       measure(2*(t-1)+1,i,:) = x(m,1);
%       measure(2*t,i,:) = x(m,2);
      X_n = [X_n; Xset{i-1}(t,:)];  
    end
    
    if length(x) < 6
        fprintf('Need atleast 6 correspondenses\n');
        continue;
    end
    [Cnew,Rnew] = PnPRANSAC(X_n, x, K);
    [Cnew, Rnew] = NonlinearPnP(X_n, x, K, Cnew, Rnew);
    
    Cset{i,1} = Cnew;
    Rset{i,1} = Rnew;
    
    x1 = Points{i-1,i}(:,1:2);
    x2 = Points{i-1,i}(:,3:4);
    
    X_new = LinearTriangulation(K, Cset{i-1}, Rset{i-1}, Cset{i}, Rset{i}, x1, x2) ;
    [X_new, id] = NonlinearTriangulation(K, Cset{i-1}, Rset{i-1}, Cset{i}, Rset{i}, x1, x2, X_new);
    X_new = X_new(id>0, :);
    x1 = x1(id>0, :);
    
    x2 = x2(id>0, :);
    indx{i-1,i} = indx{i-1,i}(id>0,:);
    n_len = len + 2* numel(indx{i-1,i}(:,:));
    measure(len+1 : n_len, Nimages) = 0;
    measure(len+1:2:n_len , i-1) = x1(:,1);
    measure(len+2:2:n_len , i-1) = x1(:,2);
    measure(len+1:2:n_len , i) = x2(:,1);
    measure(len+2:2:n_len , i) = x2(:,2);
    len = n_len;
    Xset{i} = X_new;
    X = [X;X_new];
   
    
    [t1, t2] = testX(X_new, Cset{i-1}, Rset{i-1}, Rset{i}, Cset{i}, K);
    DisplayCorrespondence(I2, x1, t1);
    DisplayCorrespondence(I3, x2, t2);

  
    %V = BuildVisibilityMatrix(Xset, Rset, Cset, i);
    
    cP{i} = K*Rset{i}*[eye(3) -Cset{i}];
    
    [cP, X] = sba_wrapper(measure, cP, X, K); 
    
    sbapath = sprintf('sba ran %d times', i-2);
    disp(sbapath);
    
end

% Plot and save the top view for 6 images
% all_X = cell2mat(Xset);
% plotfunc(all_X);
% saveas(gcf, '../Data/multiple_view_topview.fig')
