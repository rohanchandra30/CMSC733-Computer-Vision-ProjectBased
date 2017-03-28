%% Calibration: Getting World and Image Points
caliImgDir = '../Cdata';  % the folder of your calibration images in a jpg format. e.g. './Imgs'
squareSize = 21.5;  % in mm

%% Initialize calibration process
% This part takes a long time (30-45mins sometimes) to run so don't think
% your PC has crashed! Run this once and save the matrices
% this part needs the computer vision toolbox, code will error out if not available
if(~exist('Data.mat','file'))
    [x, X, imgs] = InitCalibration(caliImgDir, squareSize);
    save('Data.mat');
else
    load('Data.mat');
end

% Undistorting Camera by correcting for lens distortion
% [cameraParams, ~, ~] = estimateCameraParameters(x, X);

%% Detecting SIFT points and matching. We go straight to F matrix, E matrix detection after this.

I1 = im2double(imread('../Data/EC1.jpg'));
I1 = imrotate(I1, -90);
I2 = im2double(imread('../Data/EC2.jpg'));
I2 = imrotate(I2, -90);

% Do Undistortion
[J1, ~] = undistortImage(I1,cameraParams);
imwrite(J1,'../Data/EC1_corrected.jpg');
[J2, ~] = undistortImage(I2,cameraParams);
imwrite(J2,'../Data/EC2_corrected.jpg');
I1 = J1;
I2 = J2;
i1 = rgb2gray(I1);
i2 = rgb2gray(I2);

% Detect SIFT point
points1 = detectSURFFeatures(i1);
points2 = detectSURFFeatures(i2);

% Extract SIFt points
[f1,vpts1] = extractFeatures(i1,points1);
[f2,vpts2] = extractFeatures(i2,points2);

% Match points
indexPairs = matchFeatures(f1,f2) ;
x1 = vpts1(indexPairs(:,1));
x2 = vpts2(indexPairs(:,2));
x1 = x1.Location;
x1 = x1.Location;
x1 = double(x1);
x2 = double(x2);

% Display
figure; showMatchedFeatures(I1,I2,x1,x2, 'montage');
save('datapoints.mat', 'x1', 'x2');