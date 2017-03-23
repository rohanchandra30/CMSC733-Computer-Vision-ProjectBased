function [matches, fpoints] = get_matches(Nimages)


% Load the images and extract matching%%
for i = 1:Nimages-1
 load(sprintf('matching%d',i));
end
[match, fp]= parseMatching(Nimages, matching1, 1);
matches{1} = match;
fpoints{1} = fp; 
[match, fp]=parseMatching(Nimages, matching2, 2);
matches{2} = match;
fpoints{2} = fp;
[match, fp]= parseMatching(Nimages, matching3, 3);
matches{3} = match;
fpoints{3} = fp;
[match, fp]= parseMatching(Nimages, matching4, 4);
matches{4} = match;
fpoints{4} = fp;
[match, fp]= parseMatching(Nimages, matching5, 5);
matches{5} = match;
fpoints{5} = fp;

matches = matches';
end