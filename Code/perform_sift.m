function [x1, x2] = perform_sift()

I1 = imread('../Data/EC1_corrected.jpg');
I2 = imread('../Data/EC2_corrected.jpg');


I1 = single(rgb2gray(I1));
I2 = single(rgb2gray(I2));

[fa, da] = vl_sift(I1) ;
[fb, db] = vl_sift(I2) ;
[matches, scores] = vl_ubcmatch(da, db) ;

end