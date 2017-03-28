# CMSC733-P3
StructurefromMotion


To register *for now two* new images:

1.) Save theimages as 'EC1.jpg' and 'EC2.jpg' inthe Data folder (same folder that contains the 6 project images)

2.) Run EC_wrapper.m (This needs to be run on the server.)
        
-This file will save two things in Data.mat file. One is the K matrix, other is an object called cameraParams that is only needed for undistortion so you dont need to wory about it

-This file will save the x1, x2 (non - RANSAC) points of the 2 images in datapoints.mat

-This file will also save the two images in distortion corrected format as 'EC1_corrected.jpg' and 'EC2_corrected.jpg'

After this script finishes, copy both Data.mat and datapoints.mat back to your local system.

3.) Run EC_Code.m to reconstruction on the 2 images.

*WHENEVER PUSHING TO GIT, ALWAYS EXCLUDE DATA.MAT FROM PUSHING AS THE FILE SIZE EXCEEDS GITHUB LIMIT*


