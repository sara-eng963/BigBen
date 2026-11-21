# BigBen — Furuta pendulum

BigBen uses the team's SolidWorks URDF in Simscape Multibody. One Simulink model runs PID, pole placement, LQR, and a zero-voltage baseline on the same plant. This checkout contains the complete R2025b model set. The exports passed a smoke test in R2026a. Execution in R2025b is pending.

## Run the actual model

Use MATLAB with Simulink, Simscape, Simscape Multibody, and Control System Toolbox. Simulink Control Design is needed to regenerate the linearization.

In MATLAB, start in the repository root.

```matlab
start_bigben
open_system('BigBen_Controller_Test')
```

Run `start_bigben` once per clean MATLAB session, or after updating the plant's linearization or the controller scripts. It adds project folders to the search path, loads the actual voltage-input `linsys2.mat`, checks and orders its states through `analyze_simscape_linearization.m`, and prepares the motor parameters, pole-placement gains, LQR gains, and PID gains.

| Selector | Mode |
| --- | --- |
| 1 | PID, with integral action on arm-position error |
| 2 | Pole placement |
| 3 | LQR |
| 4 | Zero applied motor voltage |

## Test the controllers

Open `BigBen_Plant` from the top model. In `Joint_Pendulum` under **Rz → State Targets**, enable the position target in radians and set it to `bigBenUpright + deg2rad(5)` for a +5° release. Set the arm position and both joint rate targets to zero. The raw upright position is approximately +0.0487° in the imported frame.

Use the same release angle, rates, solver, duration, and actuator limits for every controller. Try +5° and −5° first. Log the arm and pendulum angles, both rates, and commanded voltage. The current controllers stabilize around upright. Swing-up from the hanging position is a later task.

Use mode 4 for open-loop checks. Zero voltage still permits back-EMF braking as the arm moves. The saved plant uses zero explicit joint damping. Record any test-only damping and motor assumptions when reporting these results.

## What the current model uses

| Source | Purpose |
| --- | --- |
| `Modeling/BigBen_Controller_Test.slx` | Current model with the four selector modes |
| `Modeling/Subsystems/BigBen_Plant.slx` | CAD-derived mechanics with torque input and position, velocity and acceleration outputs |
| `Modeling/Subsystems/Electromechanical.slx` | Converts commanded voltage to torque and uses arm velocity for back-EMF |
| `Modeling/Subsystems/Pid_Controller.slx` | Angle and rate feedback with arm-error integration |
| `Modeling/Subsystems/Lqr.slx`, `Pole_Placement.slx` | Team controller references |
| `Control/design_simscape_*.m` | Design and check gains from the actual linearization |
| `Modeling/Linearization/linsys2.mat` | Actual voltage-input linearization and upright operating-point evidence |
| `URDF/BigBen/urdf/BigBen.urdf` and `URDF/BigBen/meshes/` | Mechanical export and meshes resolved at runtime |

The state order is `[theta0; dtheta0; theta1; dtheta1]` in rad and rad/s, with controller output in V. The PID subtracts the raw upright offset before using the pendulum angle. For comparisons, check that all three controllers use the same upright origin. The exported smoke test reproduced an arm offset of about -4.86 degrees with the current LQR branch.

When dynamics change, verify the upright operating point and regenerate the linearization with the controller opened at the motor-voltage input. Keep the arm-rate back-EMF connection inside the plant and export the four named state outputs. Run the shared analysis on the new result before retuning the controllers. Preserve `linsys1.mat` as the reference-plant baseline.

## Reference work and Git

The analytical derivation, former LQR demo and reference URDF remain in Git history at 56f398d. This checkout runs the actual CAD export.

Version the executable models and scripts, required meshes and links, and the verified linearization together. Keep caches, autosaves, SolidWorks lock files, and local run evidence outside the tracked source. Use a complete and tested R2025b export for that release, with a consistent set of referenced subsystems.
