project_root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(project_root, 'src'));

X = eye(4);
Xd = eye(4);
Xdn = [1, 0, 0, 0.01;
       0, 1, 0, 0;
       0, 0, 1, 0;
       0, 0, 0, 1];

Kp = eye(6);
Ki = zeros(6);
integral = zeros(6, 1);
timestep = 0.01;

[V, Xe, next_integral] = FeedbackControl(X, Xd, Xdn, Kp, Ki, integral, timestep);

assert(isequal(size(V), [6, 1]));
assert(isequal(size(Xe), [6, 1]));
assert(isequal(size(next_integral), [6, 1]));
assert(all(isfinite(V)));
assert(all(isfinite(Xe)));
assert(all(isfinite(next_integral)));
