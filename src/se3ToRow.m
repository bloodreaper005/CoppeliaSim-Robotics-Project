function row = se3ToRow(T)
%SE3TOROW Converts a homogeneous transform to the 12-value CoppeliaSim CSV row.
row = [T(1,1:3), T(1,4), T(2,1:3), T(2,4), T(3,1:3), T(3,4)];
end
