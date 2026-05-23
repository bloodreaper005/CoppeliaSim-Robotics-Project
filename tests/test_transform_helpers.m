project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(project_root, 'src'));

T = [0, 0, 1, 0.1;
     0, 1, 0, 0.2;
    -1, 0, 0, 0.3;
     0, 0, 0, 1];

row = se3ToRow(T);
T_round_trip = row2se3(row);

assert(isequal(size(row), [1, 12]));
assert(max(abs(T(:) - T_round_trip(:))) < 1e-12);

robot_config = zeros(1, 12);
Tse = robot_state_to_end_eff_state(robot_config);

assert(isequal(size(Tse), [4, 4]));
assert(max(abs(Tse(4,:) - [0, 0, 0, 1])) < 1e-12);
assert(all(isfinite(Tse(:))));
