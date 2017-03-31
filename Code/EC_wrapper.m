%% Calibration: Getting World and Image Points
caliImgDir = '../Cdata';  % the folder of your calibration images in a jpg format. e.g. './Imgs'
squareSize = 21.5;  % in mm

%% Initialize calibration process
% This part takes a long time (30-45mins sometimes) to run so don't think
% your PC has crashed! Run this once and save the matrices
% this part needs the computer vision toolbox, code will error out if not available
if(~exist('Rohan_Data.mat','file'))
    [x, X, imgs] = InitCalibration(caliImgDir, squareSize);
    save('Rohan_Data.mat');
else
    load('Rohan_Data.mat');
end

% Undistorting Camera by correcting for lens distortion
% [cameraParams, ~, ~] = estimateCameraParameters(x, X);

%% Detecting SIFT points and matching. We go straight to F matrix, E matrix detection after this.

I1 = im2double(imread('../Data/EC1.jpg'));
% I1 = imrotate(I1, -90);
I2 = im2double(imread('../Data/EC2.jpg'));
% I2 = imrotate(I2, -90);

% Do Undistortion
[J1, ~] = undistortImage(I1,RohansCam);
imwrite(J1,'../Data/EC1_corrected.jpg');
[J2, ~] = undistortImage(I2,RohansCam);
imwrite(J2,'../Data/EC2_corrected.jpg');
