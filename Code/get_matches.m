function matches = get_matches(Nimages)


% Load the images and extract matching%%
for i = 1:Nimages-1
 load(sprintf('matching%d',i));
end
matches{1} = parseMatching(Nimages, matching1, 1);
matches{2} = parseMatching(Nimages, matching2, 2);
matches{3} = parseMatching(Nimages, matching3, 3);
matches{4} = parseMatching(Nimages, matching4, 4);
matches{5} = parseMatching(Nimages, matching5, 5);

matches = matches';
end