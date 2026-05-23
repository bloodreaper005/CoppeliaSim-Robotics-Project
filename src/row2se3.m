function T = row2se3(row)
%ROW2SE3 Converts a 12-value CoppeliaSim CSV row to a homogeneous transform.
T = [row(1), row(2), row(3), row(4);
     row(5), row(6), row(7), row(8);
     row(9), row(10), row(11), row(12);
     0, 0, 0, 1];
end
