# Mobile Manipulator Pick-and-Place in CoppeliaSim

MATLAB implementation of a feedback-control pipeline for a KUKA youBot-style mobile manipulator performing a cube pick-and-place task in CoppeliaSim.

## What It Does

- Generates an eight-segment end-effector trajectory for approach, grasp, transfer, release, and retreat.
- Simulates chassis, wheel, and arm-joint motion with velocity limiting.
- Uses feedforward plus PI feedback control on SE(3) pose error.
- Converts desired end-effector twist into wheel and joint velocities through the mobile manipulator Jacobian.
- Writes CSV outputs that can be loaded into the CoppeliaSim scene.

## Repository Structure

```text
src/       MATLAB functions for kinematics, trajectory generation, and control
scripts/   Main runnable project script
tests/     Small verification scripts
outputs/   Generated CSV files, ignored by git
docs/      Add your own report, screenshots, and notes here
```

## Requirements

- MATLAB or GNU Octave
- CoppeliaSim scene for the Modern Robotics mobile-manipulation capstone / youBot pick-and-place task

## Run

From MATLAB or Octave:

```matlab
run("scripts/run_project.m")
```

Generated files are written to `outputs/`:

- `robot_states_case.csv`
- `error.csv`
- `youbot_simulation.csv` when running `tests/test_next_state.m`

## Quick Check

```matlab
run("tests/test_next_state.m")
```

This writes a short forward-motion CSV to `outputs/youbot_simulation.csv`.

## Report

The final project report is available at [`docs/MAE-204-Final-Report.pdf`](docs/MAE-204-Final-Report.pdf).

## Notes Before Publishing

The original PDF report from the shared archive was removed because it contained other students' personal information. Add your own report and results in `docs/` if you want the GitHub repository to represent your work cleanly.

Also update this README with your own screenshots, tuned gains, CoppeliaSim scene name, and a short explanation of what you personally changed or improved.
