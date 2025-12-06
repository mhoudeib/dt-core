# Repository Guidelines

## Project Structure & Module Organization
- Core ROS packages live under `packages/` (e.g., `duckietown_demos`, `communication`, `lane_control`, `navigation`), each containing `nodes/`, `config/`, and `launch/` as needed.
- Runtime launch scripts are in `launchers/` (e.g., `default.sh`, `lane-following.sh`), used by Duckietown tooling to start specific stacks.
- Shared assets, docs, and templates sit in `assets/`, `docs/`, `libraries/`, and `dtproject/`; hardware/vision references are under `html/` and `pdf/`.
- Tests are in `tests/` (e.g., `test_state_estimator.sh`), and placeholder folders mark where to add new packages or suites.

## Build, Test, and Development Commands
- `dts devel build -f -H <hostname>`: build the workspace image for a robot from the repository root.
- `dts devel run -H <hostname>`: run the built image on the target robot; pair with `--privileged` when launchers require hardware access.
- `dts exercises build` / `dts exercises test --duckiebot_name <name>`: build or test the `mooc-exercises` integration flows when working from the exercises project.
- `dts docs build`: render the Jupyter Book in `docs/`.
- `ruff check .`: lint Python code using the repo’s rules.
- `tests/test_state_estimator.sh`: publishes IMU/Range/Twist topics for the state estimator; adjust `ROBOT_NAME` before running.

## Coding Style & Naming Conventions
- Python targets 3.12; prefer 4-space indentation, 79-character lines, 72-character docs (per `ruff.toml`).
- Follow ruff defaults (`select = ["ALL"]`); fix lint before pushing.
- Use `snake_case` for functions/variables, `PascalCase` for classes, and ROS topic namespaced by robot (e.g., `/<robot>/node/topic`).
- Keep YAML configs in package-specific `config/<node>/default.yaml`; mirror existing key names when extending.

## Testing Guidelines
- Favor ROS topic-level tests and playback scripts in `tests/`; reuse `rostopic pub` patterns from `test_state_estimator.sh`.
- Name new scripts `test_<feature>.sh`, keep them executable, and document expected topics/messages in comments.
- Run smoke tests on hardware or simulator after config changes to `launchers/` or `config/` files.

## Commit & Pull Request Guidelines
- Commit messages are short, present-tense summaries (e.g., “ground projection no longer FSM controlled”); group related changes per commit.
- PRs should include: goal/impact summary, key commands run (`dts devel build/run`, tests), config or launcher files touched, and any robot hardware dependencies.
- Link related issues, and add screenshots or logs when behavior is visual (LEDs, camera, rviz).
