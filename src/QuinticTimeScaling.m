function s = QuinticTimeScaling(Tf, t)
%QUINTICTIMESCALING Fifth-order time scaling from rest to rest.
tau = t / Tf;
s = 10 * tau^3 - 15 * tau^4 + 6 * tau^5;
end
