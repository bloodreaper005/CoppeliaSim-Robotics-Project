project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(project_root, 'src'));

Tsci = [1, 0, 0, 1;
        0, 1, 0, 1.2;
        0, 0, 1, 0.0248;
        0, 0, 0, 1];

Tscf = [0, -1, 0, 0.05;
        1,  0, 0, -1.047;
        0,  0, 1, 0.075;
        0,  0, 0, 1];

Tse = [ 0, 0, 1, 0;
        0, 1, 0, 0;
       -1, 0, 0, 0.5;
        0, 0, 0, 1];

Tces = [-1, 0, 0, 0;
         0, 1, 0, 0;
         0, 0,-1, 0.1;
         0, 0, 0, 1];

Tceg = [-1, 0, 0, 0;
         0, 1, 0, 0;
         0, 0,-1,-0.01;
         0, 0, 0, 1];

k = 1;
trajectory = TrajectoryGenerator(Tse, Tsci, Tscf, Tceg, Tces, k);
segment_times = [10; 3; 0.625; 3; 10; 3; 0.625; 3];
expected_rows = sum(round(segment_times * k * 100));

assert(isequal(size(trajectory), [expected_rows, 13]));
assert(all(trajectory(:, 13) == 0 | trajectory(:, 13) == 1));
assert(all(isfinite(trajectory(:))));
