function result = NearZero(value)
%NEARZERO Returns true when a scalar is close enough to zero for robotics math.
result = abs(value) < 1e-6;
end
