project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(project_root, 'src'));

fprintf('Running MATLAB validation scripts...\n');

run(fullfile(project_root, 'tests', 'test_next_state.m'));
run(fullfile(project_root, 'tests', 'test_transform_helpers.m'));
run(fullfile(project_root, 'tests', 'test_trajectory_generator.m'));
run(fullfile(project_root, 'tests', 'test_feedback_control_shapes.m'));

fprintf('All validation scripts completed.\n');
