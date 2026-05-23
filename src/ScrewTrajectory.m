function traj = ScrewTrajectory(Xstart, Xend, Tf, N, method)
%SCREWTRAJECTORY Generates a screw-motion trajectory between two SE(3) poses.
timegap = Tf / (N - 1);
traj = cell(1, N);

for i = 1:N
    if method == 3
        s = CubicTimeScaling(Tf, timegap * (i - 1));
    else
        s = QuinticTimeScaling(Tf, timegap * (i - 1));
    end
    traj{i} = Xstart * MatrixExp6(MatrixLog6(TransInv(Xstart) * Xend) * s);
end
end
