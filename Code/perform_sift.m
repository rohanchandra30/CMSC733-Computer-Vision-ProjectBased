function [x1, x2] = perform_sift(I1, I2)


I1 = single(rgb2gray(I1));
I2 = single(rgb2gray(I2));

[fa, da] = vl_sift(I1) ;
[fb, db] = vl_sift(I2) ;
[matches, scores] = vl_ubcmatch(da, db) ;

xa = fa(1,matches(1,:))' ;
xb = fb(1,matches(2,:))' ;
ya = fa(2,matches(1,:))' ;
yb = fb(2,matches(2,:))' ;

x1 = [xa ya];
x2 = [xb yb];

dispMatchedFeatures(I2,I3,x1,x2, 'montage');
end