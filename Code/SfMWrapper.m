%% Good Luck, Have fun!
Images = [];
Imagepath = sprintf('../Data');
addpath(Imagepath);
listing = dir(Imagepath);
Nimages = 6;

%%  Taken from calibration.txt %%
K = [568.996140852 0 643.21055941; 0 568.988362396 477.982801038; 0 0 1];

%% Load the images and extract matching%%
for i = 1:Nimages-1
 load(sprintf('matching%d',i));
end
matches = cell(Nimages-1,1);
matches{1} = parseMatching(Nimages, matching1, 1);
matches{2} = parseMatching(Nimages, matching2, 2);
matches{3} = parseMatching(Nimages, matching3, 3);
matches{4} = parseMatching(Nimages, matching4, 4);
matches{5} = parseMatching(Nimages, matching5, 5);

% -------------------------------------------------------Rohan's Commit-----------------------------------------------------------------------
F = cell(Nimages-1, Nimages);
idx = cell(Nimages-1, Nimages);
y1 = cell(Nimages-1, Nimages);
y2 = cell(Nimages-1, Nimages);

for i=1:Nimages-1
   for j = i+1:Nimages
     if ~isempty(matches{i,1}{j,1})
       [a1,a2,id,f] = GetInliersRANSAC(matches{i,1}{j,1}(:,1),matches{i,1}{j,1}(:,2));     
       y1{i,j} = a1;
       y2{i,j} = a2;
       idx{i,j} = id;
       F{i,j} = f;
     end 
   end    
end    

% Part 5

E = EssentialMatrixFromFundamentalMatrix(F{1,2},K);
[Cset,Rset] = ExtractCameraPose(E); 
