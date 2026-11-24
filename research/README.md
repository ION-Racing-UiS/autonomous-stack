# Research

Experiments, test scripts, notebooks and models that are not (yet) a ROS 2 node.
Code that runs in the stack lives in `src/ion_<block>/`, not here.

## Layout

| Folder | Owner | Put here |
| --- | --- | --- |
| `perception/` | Perception | Detector experiments, tuning sweeps, plots |
| `state_estimation/` | State estimation | EKF prototypes, noise models |
| `planning/` | Planning | Planner prototypes |
| `control/` | Control | Simulink models, MPC experiments |
| `mechanical/` | Mechanical | Mounting sketches, sensor placement studies |
| `integration/` | Integration | Sim setup, bringup experiments |
| `literature/` | Everyone | Links to papers (see `literature/README.md`) |

Add subfolders inside your block's folder as you like.

## Rules

- **No paper PDFs.** This repo is public, and most papers may not be redistributed. Add a link or DOI to `literature/README.md` instead.
- **No large data files.** Rosbags (`.bag`, `.db3`, `.mcap`) and other recordings are git-ignored. Share them on the team drive and link them in your folder's README.
- **Keep it runnable.** A short README per experiment: what it tests, how to run it, what you found.
