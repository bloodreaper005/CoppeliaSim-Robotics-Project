# Reproducibility

This page lists the steps and evidence needed to rerun the project when MATLAB and CoppeliaSim are available.

## Required Software

- MATLAB or GNU Octave with support for the functions used in `src/`
- CoppeliaSim with the mobile-manipulation / youBot pick-and-place scene
- A CSV import workflow for robot state playback in CoppeliaSim

## Run Order

1. Open MATLAB or Octave at the repository root.
2. Run the validation scripts:

   ```matlab
   run("tests/run_all_tests.m")
   ```

3. Generate the pick-and-place CSV:

   ```matlab
   run("scripts/run_project.m")
   ```

4. Load `outputs/robot_states_case.csv` into the CoppeliaSim scene.
5. Compare the generated plots with the report figures in `docs/results/`.

## Expected Outputs

- `outputs/robot_states_case.csv`
- `outputs/error.csv`
- MATLAB figures for end-effector error and manipulability

## Current Verification Status

The repository cleanup verified file structure, attribution, privacy, README links, and static MATLAB function references. Runtime verification was not performed in the cleanup environment because MATLAB and CoppeliaSim were unavailable.

## Recommended Future Evidence

- Add the exact MATLAB version used.
- Add the exact CoppeliaSim version used.
- Add the scene file if it is allowed to be shared.
- Add a short GIF or MP4 demo in `docs/`.
