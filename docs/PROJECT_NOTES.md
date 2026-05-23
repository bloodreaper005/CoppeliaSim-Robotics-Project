# Project Notes

This project implements a mobile-manipulation pick-and-place controller for a youBot-style robot. The implementation is split into trajectory generation, kinematic state propagation, Jacobian-based velocity mapping, and feedforward-plus-feedback control.

## Controller Tuning

The final report evaluates three controller settings:

| Scenario | Gains | Purpose |
|---|---|---|
| Best case | `Kp = 4.5 * I`, `Ki = 0` | Stable tracking with minimal overshoot. |
| Overshoot | `Kp = 5 * I`, `Ki = 0.5 * I` | Demonstrates the effect of aggressive integral action. |
| New task | `Kp = 3 * I`, `Ki = 0` | Tests adaptation to a modified cube start/end position. |

## Main Takeaways

- Quintic time scaling helps smooth acceleration and deceleration between trajectory segments.
- Integral gain can reduce long-term bias, but excessive integral action causes overshoot and rougher motion.
- Low velocity limits can increase tracking error because the robot cannot keep up with the feedforward trajectory.
- Manipulability changes are most noticeable when the arm approaches extended or less dexterous configurations.

## Future Improvements

- Add anti-windup for the integral term.
- Add a damped pseudoinverse option near singularities.
- Add exact MATLAB and CoppeliaSim version information after rerunning the project.
- Add the CoppeliaSim scene file if it is allowed to be shared.
