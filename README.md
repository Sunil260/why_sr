## WHY-SR

WHY-SR is a Python-based search-and-retrieve robotics project for a 2-wheel drive platform.
The codebase is organized around perception, control, hardware drivers, and a mission state machine.

The robot workflow is designed around tasks like:
- Following a red guide path
- Detecting and aligning to a target
- Approaching and picking up the target with a claw
- Navigating to a green safe zone
- Dropping the target and returning

## Tech Stack

- Python 3.11+
- OpenCV (`opencv-python`) for vision and color-based detection
- `gpiozero` and Raspberry Pi PWM/GPIO libraries for hardware control on Linux/Raspberry Pi
- `uv` for dependency and environment management

## Setup (Recommended with uv)

This project is configured with [pyproject.toml](pyproject.toml), so `uv` is the easiest way to set it up.

1. Install `uv` (if needed):

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

2. From the project root, create and sync the environment:

```bash
uv sync
```

3. Run scripts inside the managed environment:

```bash
uv run python <script>.py
```

Examples:

```bash
uv run python test/test_camera.py
uv run python test/test_red_detection.py
uv run python test/test_drivebase.py
```

## Platform Notes

- On macOS/dev machines, vision code will run, but GPIO/PWM hardware access is not expected to work.
- On Raspberry Pi/Linux, platform-specific dependencies in [pyproject.toml](pyproject.toml) are enabled via environment markers.
- If you only want to work on perception algorithms, camera and OpenCV tests are the best starting point.

## Project Structure

Top-level layout:

```text
.
├── controllers.py            # Control primitives and task-level controllers
├── drivebase.py              # Differential drive wrapper around motor driver hardware
├── manipulator.py            # Claw/servo manipulation logic
├── perception.py             # Camera abstraction + vision pipelines
├── state_machine.py          # High-level mission state transitions
├── main.py                   # Intended runtime entry (currently a placeholder)
├── test.py                   # Legacy local test script
├── pyproject.toml            # Project metadata and dependencies
├── hardware/
│   ├── beam_break_sensor.py  # Beam-break sensor wrapper
│   ├── dc_motor_driver.py    # L298 motor driver control
│   └── servo_driver.py       # Continuous servo PWM helper
├── test/                     # Focused functional and subsystem tests
├── archive/                  # Older control/perception experiments
└── utils/
    └── tuner_ui.py           # Utility tooling (parameter tuning UI)
```

### Core Modules

- [controllers.py](controllers.py):
  Contains reusable controller components (`PDController`) and task-specific controllers such as line following, alignment, approach, and turn-until-line.

- [perception.py](perception.py):
  Implements camera capture (`OpenCVCamera`) and color-based detectors for red line following, green safe-zone detection, and target/lego detection.

- [state_machine.py](state_machine.py):
  Implements a mission FSM (`StateMachine`) with states for acquiring line, following path, aligning, approaching, pickup, drop-off, and recovery.

- [drivebase.py](drivebase.py), [manipulator.py](manipulator.py), [hardware/](hardware):
  Hardware abstraction layer for movement and end-effector control.

## Running Tests / Checks

The repository currently uses script-style tests in [test/](test) rather than a strict single test runner.
Run individual tests with `uv run`:

```bash
uv run python test/test_camera.py
uv run python test/test_green_detection.py
uv run python test/test_target_detection.py
```

For hardware tests, run them on Raspberry Pi with proper wiring and permissions.

## Development Tips

- Keep perception and control modules hardware-agnostic where possible.
- Use test scripts in [test/](test) to iterate quickly on individual subsystems.
- If adding dependencies, update [pyproject.toml](pyproject.toml), then run:

```bash
uv sync
```

