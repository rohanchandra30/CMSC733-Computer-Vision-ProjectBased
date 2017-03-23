function points = get_point_cell(Nimages)



points = cell(6,6);

matches = get_matches(Nimages);

[x1,x2] = get_points(matches{1}{2});
points{1,2} = [x1, x2];
[x1,x3] = get_points(matches{1}{3});
points{1,3} = [x1, x3];
[x1,x4] = get_points(matches{1}{4});
points{1,4} = [x1, x4];
[x2,x3] = get_points(matches{2}{3});
points{2,3} = [x2, x3];
[x2,x4] = get_points(matches{2}{4});
points{2,4} = [x2, x4];
[x3,x4] = get_points(matches{3}{4});
points{3,4} = [x3, x4];
[x3,x5] = get_points(matches{3}{5});
points{3,5} = [x3, x5];
[x3,x6] = get_points(matches{3}{6});
points{3,6} = [x3, x6];
[x4,x5] = get_points(matches{4}{5});
points{4,5} = [x4, x5];
[x4,x6] = get_points(matches{4}{6});
points{4,6} = [x4, x6];
[x5,x6] = get_points(matches{5}{6});
points{5,6} = [x5, x6];


end