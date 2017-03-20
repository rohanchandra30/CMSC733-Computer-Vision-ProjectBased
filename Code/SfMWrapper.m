%% Good Luck, Have fun!
Images = [];
Imagepath = sprintf('../Data');
addpath(Imagepath);
listing = dir(Imagepath);
Nimages = 6;

%%  Taken from calibration.txt %%
K = [568.996140852 0 643.21055941; 0 568.988362396 477.982801038; 0 0 1];

%% Load the images and extract matching%%


for i = 1:Nimages
    %Images(i) = imread(sprintf('%s/%d.jpg',Imagepath, i));
    filename = fullfile(Imagepath,sprintf('matching%d.txt',i));
    fileID = fopen(filename);
    nfeatures = fscanf(fileID,'nFeatures: %d');
    
    if i ~= Nimages
      nFeatures = textread(sprintf('%s/matching%d.txt', Imagepath, i), 'nnFeatures: %d');
      field1 = 'nFeatures';
      value1 = nFeatures;
      for j = 1:nFeatures
          [nMatches r g b x y] = textread(sprintf('%s/matching%d.txt', Imagepath, i), '%d %d %d %d %f %f');
          
      end   
    end  
   fclose(fileID);  
end    



