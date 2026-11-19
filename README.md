# BigBen — Furuta Inverted Pendulum

MATLAB/Simulink implementation of a Furuta rotary inverted pendulum for the MCT411 Modern Control project.

The project uses **Simscape Multibody** for nonlinear mechanical simulation, an electromechanical DC motor model for actuation, and MATLAB's Control System Toolbox for linearization analysis and controller design.

The implementation follows a modular architecture based on **Simulink Referenced Subsystems**, allowing multiple controllers to operate on the same mechanical and electromechanical models without duplicating their implementations.

## 1. Quick Start

This section describes how to initialize MATLAB and run the existing open-loop or pole-placement simulation.

### 1.1 Requirements

The Simscape implementation requires:

- MATLAB
- Simulink
- Simscape
- Simscape Multibody
- Control System Toolbox
- Simulink Control Design for generating new linearizations

Symbolic Math Toolbox is additionally required for the separate analytical Euler–Lagrange implementation.

### 1.2 MATLAB Initialization

Open MATLAB and set the **Current Folder** to the repository root (`ProjectRepo`).

Execute the following commands in the MATLAB Command Window:

```matlab
%% BigBen - MATLAB Initialization

% Project root
root = pwd;

% Add required project directories
addpath( ...
    fullfile(root, 'Modeling'), ...
    fullfile(root, 'Modeling', 'Subsystems'), ...
    fullfile(root, 'Control'), ...
    fullfile(root, 'model-notours'));

% Load the saved Simscape linearization
load(fullfile(root, 'Modeling', 'Linearization', ...
    'linsys1.mat'), 'linsys1');

% Initialize motor parameters and extract A, B, C, D
analyze_simscape_linearization;

% Calculate pole-placement feedback gain
design_simscape_pole_placement;

% Open the main simulation
open_system('furuta_urdf_test');
```

The initialization provides the following variables:

| Variable | Description |
|---|---|
| `motor` | Electromechanical motor parameters |
| `linsys1` | Saved Simscape linearization |
| `A`, `B`, `C`, `D` | State-space matrices in the project's standard state order |
| `K_pp_sim` | Pole-placement feedback gain |

The saved linearization allows the existing controller to be executed without repeating the Model Linearizer procedure in every MATLAB session.

### 1.3 Running the Simulation

The main model is:

`Modeling/furuta_urdf_test.slx`

The model contains a Manual Switch used to select the operating mode.

| Mode | Motor input | Description |
|---|---|---|
| Open Loop | Constant 0 V | Uncontrolled mechanical response with motor back-EMF effects |
| Pole Placement | \(V_m=-K_{pp,sim}x\) | Closed-loop stabilization around the upright equilibrium |

To execute a simulation:

1. Select the operating mode using the Manual Switch.
2. Configure the initial joint conditions if required (Section 5).
3. Set the simulation stop time.
4. Update the model using **Ctrl + D**.
5. Run the simulation.
6. Inspect the joint states and motor-voltage response using the available Scopes.

A standard stabilization test uses an initial pendulum displacement of **5° from upright** with both joints initially stationary.

## 2. Project Structure

```text
ProjectRepo/
│
├── Modeling/
│   ├── furuta_urdf_test.slx
│   ├── furuta_simscape.slx
│   ├── analyze_simscape_linearization.m
│   │
│   ├── Subsystems/
│   │   ├── Plant.slx
│   │   ├── Electromechanical.slx
│   │   └── Pole_Placement.slx
│   │
│   ├── Linearization/
│   │   └── linsys1.mat
│   │
│   └── Analytical (lagrangian)/
│       ├── symbolic_lagrangian.m
│       ├── furuta_parameters.m
│       ├── linearize_upright.m
│       ├── run_open_loop.m
│       └── ...
│
├── Control/
│   ├── design_simscape_pole_placement.m
│   └── Analytical (lagrangian/
│       └── ... Analytical controller files
│
├── model-notours/
│   ├── model.urdf
│   └── meshes/
│
├── CAD/
├── Resources/
└── README.md
```

### 2.1 Main Files

| File | Responsibility |
|---|---|
| `furuta_urdf_test.slx` | Main Simscape simulation and system integration |
| `Plant.slx` | Mechanical assembly, revolute joints, and state sensing |
| `Electromechanical.slx` | DC motor voltage-to-torque conversion |
| `Pole_Placement.slx` | Pole-placement state-feedback implementation |
| `linsys1.mat` | Saved linearization of the reference Simscape model |
| `analyze_simscape_linearization.m` | Motor initialization, state-space extraction, and open-loop analysis |
| `design_simscape_pole_placement.m` | Pole-placement design and closed-loop eigenvalue verification |
| `model.urdf` | Source mechanical model imported into Simscape |

### 2.2 Modelling Approaches

The repository contains two separate modelling approaches.

**Simscape implementation — Primary**

The Simscape implementation represents the nonlinear mechanical system using an imported URDF and Simscape Multibody blocks. Controllers are developed using a linearization of this physical simulation model.

**Analytical implementation — Reference**

The analytical implementation derives the equations of motion using the Euler–Lagrange method and provides an independent mathematical model for analysis and comparison.

The analytical and Simscape models must not be assumed to have identical physical parameters.

Controller gains derived for one model must not be used on the other without verification.

## 3. System Architecture

The main simulation consists of three reusable components:

```text
               Controller
             (Pole Placement,
              LQR, PID, ...)
                   |
               Motor Voltage
                   |
                   v
           Electromechanical <------ Arm Angular Velocity
                 Model
                   |
               Motor Torque
                   |
                   v
             Mechanical Plant
           (Simscape Multibody)
                   |
             Measured States
                   |
                   +--------------------> Controller
```

### 3.1 Standard State Convention

The project uses the state vector:

\[
x=
\begin{bmatrix}
\theta_0\\
\dot{\theta}_0\\
\theta_1\\
\dot{\theta}_1
\end{bmatrix}
\]

where:

| State | Description | Unit |
|---|---|---|
| \(\theta_0\) | Rotary-arm angle | rad |
| \(\dot{\theta}_0\) | Rotary-arm angular velocity | rad/s |
| \(\theta_1\) | Pendulum angle | rad |
| \(\dot{\theta}_1\) | Pendulum angular velocity | rad/s |

The modified reference URDF uses:

- \(\theta_1=0^\circ\): Upright position
- \(\theta_1=180^\circ\): Downward position

All controller implementations must follow the same state ordering and angle convention.

### 3.2 Mechanical Plant

**File:** `Modeling/Subsystems/Plant.slx`

The Plant subsystem contains the imported mechanical assembly, revolute joints, state sensors, and the required Simscape signal-conversion blocks.

| Direction | Signal | Unit |
|---|---|---|
| Input | `torque` | N·m |
| Output | `theta0` | rad |
| Output | `dtheta0` | rad/s |
| Output | `theta1` | rad |
| Output | `dtheta1` | rad/s |

The Plant subsystem represents mechanical dynamics only. Motor-voltage calculations and controller logic are implemented separately.

### 3.3 Electromechanical Model

**File:** `Modeling/Subsystems/Electromechanical.slx`

The electromechanical subsystem implements a simplified DC motor model:

\[
\tau_m=\frac{K_t}{R_m}(V_m-K_e\dot{\theta}_0)
\]

where:

- \(V_m\): Applied motor voltage
- \(R_m\): Armature resistance
- \(K_t\): Torque constant
- \(K_e\): Back-EMF constant
- \(\tau_m\): Applied motor torque

Current preliminary simulation parameters:

| Parameter | Value |
|---|---:|
| \(R_m\) | 8 Ω |
| \(K_t\) | 0.05 N·m/A |
| \(K_e\) | 0.05 V·s/rad |

These values are preliminary simulation assumptions, not validated hardware specifications.

Motor inductance is neglected. Physical driver limitations and voltage saturation have not yet been validated.

The subsystem interface is:

| Input | Output |
|---|---|
| `Voltage` | `Torque` |
| `dtheta0` | |

The arm-velocity input is required for the back-EMF calculation and must remain connected in every simulation.

### 3.4 Controller Subsystems

Controller implementations are stored independently as referenced subsystems.

The existing controller is:

`Modeling/Subsystems/Pole_Placement.slx`

Its feedback law is:

\[
V_m=-K_{pp,sim}x
\]

Future controllers should preserve the standardized input/output interface so they can be integrated without modifying the shared Plant or Electromechanical subsystems.

## 4. Developing and Integrating a New Controller

This section defines the standard workflow for implementing additional controllers, including LQR, PID, and other control strategies.

**All controller implementations must reuse the existing Plant and Electromechanical references.**

### 4.1 Create a New Simulation Model

1. Open Simulink and create a **Blank Model**.
2. Save it using a controller-specific filename, such as `LQR_Simscape.slx` or `PID_Simscape.slx`.
3. Keep the existing `furuta_urdf_test.slx` model unchanged.

Independent top-level models allow different control strategies to be developed and tested without interfering with the verified implementation.

### 4.2 Insert the Shared Mechanical Plant

The existing mechanical subsystem is stored at:

`Modeling/Subsystems/Plant.slx`

To include it:

1. Open the Simulink Library Browser.
2. Navigate to **Simulink → Ports & Subsystems**.
3. Insert a **Subsystem Reference** block.
4. Set its referenced subsystem file to `Plant.slx`.

Alternatively, copy the existing Plant reference block from `furuta_urdf_test.slx` into the new model. This preserves the reference link.

Do not copy the individual internal mechanical blocks into each controller simulation.

### 4.3 Insert the Shared Electromechanical Model

Repeat the same procedure using:

`Modeling/Subsystems/Electromechanical.slx`

Connect the shared components as follows:

| Source | Destination |
|---|---|
| Electromechanical `Torque` | Plant `torque` |
| Plant `dtheta0` | Electromechanical `dtheta0` |

These connections are required regardless of the controller being implemented.

### 4.4 Create the Controller Subsystem

Insert a regular Simulink **Subsystem** block and name it according to the controller, for example `LQR_Controller`.

Create the following interface:

**Inputs**

| Port | Signal |
|---|---|
| 1 | `theta0` |
| 2 | `dtheta0` |
| 3 | `theta1` |
| 4 | `dtheta1` |

**Output**

| Port | Signal |
|---|---|
| 1 | `Voltage` |

The controller should receive measured states and produce the motor-voltage command.

Its internal implementation depends on the selected control strategy.

#### LQR Implementation

Use the Simscape-derived state-space matrices:

```matlab
K_lqr = lqr(A, B, Q, R);
```

The feedback law is:

\[
V_m=-K_{\mathrm{LQR}}x
\]

Inside the subsystem:

1. Combine the four measured states into a vector using a Mux.
2. Add a Gain block.
3. Set the gain to `-K_lqr`.
4. Configure multiplication as **Matrix (K*u)**.
5. Connect the output to the `Voltage` Outport.

The weighting matrices `Q` and `R` must be selected, justified, and documented in a separate controller-design script.

#### PID Implementation

Use Simulink's built-in **PID Controller** blocks.

The controller architecture must account for the fact that the Furuta pendulum has two rotational degrees of freedom but only one actuator.

The selected architecture and tuning procedure should be documented independently.

PID gains must be designed using the selected plant model and validated on the complete system. Stabilizing the pendulum angle alone does not necessarily stabilize the rotary arm.

### 4.5 Convert the Controller to a Referenced Subsystem

Once the controller implementation is complete:

1. Return to the top-level Simulink model.
2. Right-click the controller subsystem.
3. Select **Convert to → Referenced Subsystem**.
4. Save it using a unique name, such as `LQR_Controller.slx`.
5. Store it under `Modeling/Subsystems/`.

The controller now has a separate source file and can be reused in additional simulation models.

### 4.6 Complete the Feedback Connections

Connect the Plant outputs to the corresponding controller inputs.

| Plant Output | Controller Input |
|---|---|
| `theta0` | `theta0` |
| `dtheta0` | `dtheta0` |
| `theta1` | `theta1` |
| `dtheta1` | `dtheta1` |

Then connect:

**Controller `Voltage` → Electromechanical `Voltage`**

The complete closed-loop system is now assembled.

Before running, calculate or load the controller's required gains and verify that they correspond to the selected mechanical plant.

## 5. Configuring Initial Conditions

Initial joint conditions are configured inside the mechanical Plant reference.

### 5.1 Accessing the Revolute Joints

1. Open the top-level Simulink model.
2. Open the referenced **Plant** subsystem.
3. Locate the required Revolute Joint:
   - `base_to_arm`: Rotary-arm joint
   - `arm_to_pendulum`: Pendulum joint
4. Double-click the joint.
5. Navigate to **Z Revolute Primitive (Rz) → State Targets**.

Enable the required position and velocity targets and set their values.

Use **High (desired)** priority where appropriate, subject to the assembly constraints.

### 5.2 Standard Initial Conditions

**Upright Equilibrium**

| Joint | Initial Position | Initial Velocity |
|---|---|---|
| `base_to_arm` | 0° | 0°/s |
| `arm_to_pendulum` | 0° | 0°/s |

**Stabilization Test — 5° Disturbance**

| Joint | Initial Position | Initial Velocity |
|---|---|---|
| `base_to_arm` | 0° | 0°/s |
| `arm_to_pendulum` | 5° | 0°/s |

After changing the State Targets:

1. Apply the joint settings.
2. Save the required model changes.
3. Update the simulation using **Ctrl + D**.
4. Run the selected test.

State Targets guide the mechanism's initial configuration. They do not hold the joints at those positions during simulation.

### 5.3 Shared-Reference Considerations

`Plant.slx` is used by multiple top-level simulations.

Modifying and saving the joint State Targets inside the shared referenced subsystem changes the configuration used by other models referencing the same source.

Consequently, experimental changes to joint initialization must be coordinated between team members.

**Recommended future improvement:** Parameterize the joint State Targets and supply experiment-specific values through model or subsystem parameters. This would allow each simulation to define independent initial conditions without modifying the shared Plant source file.

Until this is implemented, changes to shared State Targets should be treated as changes to a common project component.

## 6. Linearization and State-Space Analysis

### 6.1 Existing Simscape Linearization

The reference Simscape model was linearized around the upright equilibrium using **Simulink Model Linearizer**.

The linear model has the form:

\[
\dot{x}=Ax+BV_m
\]

\[
y=Cx+DV_m
\]

The result is saved as:

`Modeling/Linearization/linsys1.mat`

The MATLAB variable containing the saved state-space model is `linsys1`.

### 6.2 Analyzing the Saved Linearization

Load the saved result:

```matlab
load('Modeling/Linearization/linsys1.mat', 'linsys1');
```

Then execute:

```matlab
analyze_simscape_linearization;
```

The script:

1. Initializes the preliminary motor parameters.
2. Extracts the state-space matrices from `linsys1`.
3. Reorders the states into the standard project convention.
4. Computes the open-loop eigenvalues.
5. Evaluates controllability and observability.

The original Simscape linearization may use a different internal state order.

The analysis constructs a state transformation:

\[
x=Tz
\]

where \(z\) is the original Simscape state vector and \(x\) follows the project convention.

The matrices are transformed using:

\[
A=TA_{\mathrm{sim}}T^{-1}
\]

\[
B=TB_{\mathrm{sim}}
\]

The resulting matrices are made available as `A`, `B`, `C`, and `D`.

### 6.3 Generating a New Linearization

Repeat linearization whenever changes to the mechanical or electromechanical dynamics invalidate the saved model.

**Procedure**

1. Open the relevant top-level Simulink model.
2. Configure the joints at the intended equilibrium.
3. Set the equilibrium motor-voltage input, normally 0 V for the upright operating point.
4. Verify that the selected configuration is a valid equilibrium.
5. Configure the linear analysis points:
   - Input: Motor voltage entering the Electromechanical subsystem
   - Outputs: `theta0`, `dtheta0`, `theta1`, and `dtheta1`
6. Ensure the controller is excluded from the plant linearization by using an appropriate open-loop analysis configuration.
7. Open **Apps → Model Linearizer**.
8. Select **Model I/Os** and the verified operating point.
9. Execute the linearization.
10. Export the resulting state-space model to the MATLAB workspace.

The existing reference linearization uses the variable name `linsys1`.

Save it using:

```matlab
save('Modeling/Linearization/linsys1.mat', 'linsys1');
```

The saved linear model must correspond to the physical parameters and actuator configuration used in the simulation.

### 6.4 Controller Design Using the Linear Model

Controllers designed from the linearization must use matrices derived from the same plant configuration.

Controller design and plant analysis are maintained in separate MATLAB scripts.

This separation allows the same linearized plant to be used for different control strategies without duplicating the mechanical modelling work.

## 7. Pole-Placement Controller

### 7.1 Design Script

**File:** `Control/design_simscape_pole_placement.m`

The script uses the linearized state-space matrices obtained from the Simscape model.

It calculates the state-feedback gain using:

\[
K=\operatorname{place}(A,B,P)
\]

where \(P\) contains the desired closed-loop poles.

The current design specifications are:

| Specification | Value |
|---|---|
| Settling-time design target | 2 s |
| Damping ratio | 0.5 |
| Dominant poles | \(-2\pm3.4641j\) |
| Additional poles | \(-40,-50\) |

The script verifies closed-loop stability using:

\[
A_{\mathrm{cl}}=A-BK
\]

The resulting gain is saved in the MATLAB workspace as `K_pp_sim`.

### 7.2 Simulink Implementation

**File:** `Modeling/Subsystems/Pole_Placement.slx`

The controller receives the four measured states and implements:

\[
V_m=-K_{pp,sim}x
\]

The feedback gain is applied using a Simulink Gain block configured for matrix multiplication.

### 7.3 Verification

The design script verifies that the eigenvalues of \(A-BK\) match the desired closed-loop poles.

The controller has also been tested in the nonlinear Simscape simulation from an initial pendulum displacement.

The linear pole test demonstrates local linear closed-loop stability. Nonlinear simulations are required to assess the response beyond that approximation.

Hardware voltage limits, unmodelled disturbances, and robustness margins remain subjects for further validation.

## 8. Migrating to a New URDF Model

**Status: Planned development workflow.**

The current mechanical simulation is based on an imported reference URDF. When the final project CAD model becomes available, it must be imported and validated as a separate mechanical plant.

The original reference implementation and its linearization must be preserved for comparison and reproducibility.

### 8.1 Migration Architecture

| Component | Existing Reference | New Implementation |
|---|---|---|
| URDF source | `model-notours/model.urdf` | `model-main/model.urdf` |
| Mechanical subsystem | `Plant.slx` | `Plant_Main.slx` |
| Main simulation | `furuta_urdf_test.slx` | `furuta_main.slx` |
| Saved linearization | `linsys1.mat` | `linsys2.mat` |
| Motor subsystem | `Electromechanical.slx` | Reused if specifications are unchanged |
| Controller gains | Reference-plant gains | Recalculated for new plant |

The new filenames represent the intended migration structure and are not part of the current verified implementation.

### 8.2 Import the New URDF

Store the new source model and its meshes in a dedicated directory:

```text
ProjectRepo/
├── model-notours/
│   ├── model.urdf
│   └── meshes/
│
└── model-main/
    ├── model.urdf
    └── meshes/
```

Verify that the URDF contains the correct:

- Link geometry and mesh references
- Joint origins and rotation axes
- Link masses
- Centers of mass
- Inertia tensors
- Mechanical assembly relationships

Import the new model into Simscape:

```matlab
smimport(fullfile('model-main', 'model.urdf'), ...
    'ModelName', 'furuta_main_imported');
```

This creates an independent imported mechanical model.

Inspect the new assembly in Multibody Explorer and verify the physical orientation, joint motions, and gravity response.

The standard convention remains:

\[
\theta_1=0^\circ \quad \text{at upright}
\]

Any difference in joint reference configuration or rotation sign must be resolved before controller design.

### 8.3 Create `Plant_Main.slx`

Configure the imported assembly as a mechanical plant with externally applied motor torque.

Required modifications include:

1. Configure the rotary-arm joint to receive torque through its actuation input.
2. Keep the pendulum joint passive.
3. Enable position and velocity sensing for both joints.
4. Add the required Simulink-PS and PS-Simulink converters.
5. Expose the standardized input and output ports.

The interface must remain:

| Direction | Signal |
|---|---|
| Input | `torque` |
| Output | `theta0` |
| Output | `dtheta0` |
| Output | `theta1` |
| Output | `dtheta1` |

Group the configured mechanical assembly into a subsystem and convert it to a Referenced Subsystem.

Save as:

`Modeling/Subsystems/Plant_Main.slx`

Do not overwrite `Plant.slx`.

### 8.4 Create `furuta_main.slx`

Create a new top-level simulation containing:

- `Plant_Main.slx`
- `Electromechanical.slx`
- The required controller reference

Connect the subsystems using the standard architecture described in Section 3.

If the new design retains the same motor, the existing Electromechanical subsystem may be reused.

If the motor specifications differ, the motor model or its parameter configuration must be updated and documented. Changes to the shared motor implementation must not silently invalidate existing simulations.

Before controller integration, run open-loop tests and verify the new assembly's mechanical behavior.

### 8.5 Obtain a New Linearization

The new mechanical model may have different mass distribution, inertia, geometry, and dynamic coupling. The existing state-space matrices and controller gains must therefore not be assumed valid.

Configure the new model at the upright equilibrium and repeat the linearization procedure from Section 6.

Export the new linear model as `linsys2`.

Save it separately:

```matlab
save('Modeling/Linearization/linsys2.mat', 'linsys2');
```

The linearization directory will then contain:

```text
Modeling/
└── Linearization/
    ├── linsys1.mat
    └── linsys2.mat
```

**Important:** The current `analyze_simscape_linearization.m` expects an input variable named `linsys1`.

Before adopting the new plant, the analysis should be updated to accept a selected linearized model independently of its filename or workspace variable name.

Until that update is implemented, the new linearization must not be treated as a drop-in replacement for the existing analysis workflow.

### 8.6 Redesign the Controllers

For every controller intended for use with `Plant_Main.slx`:

1. Load the new linearized model.
2. Extract and verify the state-space matrices.
3. Confirm the state-ordering and angle conventions.
4. Analyze open-loop stability and controllability.
5. Recalculate or retune the controller gains.
6. Verify the linear closed-loop response.
7. Test the nonlinear closed-loop behavior using `furuta_main.slx`.

Controller gains for different plants must be stored independently.

Recommended naming:

| Variable | Purpose |
|---|---|
| `K_pp_sim` | Existing reference-plant pole-placement gain |
| `K_pp_main` | New mechanical plant pole-placement gain |
| `K_lqr_main` | New mechanical plant LQR gain |

The same principle applies to PID coefficients and other controller parameters.

### 8.7 Migration Acceptance Criteria

The new mechanical implementation is ready for controller development when:

- The imported geometry and physical parameters have been checked.
- Joint axes and angle conventions are consistent.
- `Plant_Main.slx` compiles successfully.
- The motor subsystem connects correctly.
- Open-loop behavior has been verified.
- The upright equilibrium has been checked.
- `linsys2.mat` has been generated and saved.
- State-space ordering, eigenvalues, and controllability have been verified.

The migration is considered validated for a particular controller only after linear and nonlinear closed-loop testing.

## 9. Simulation and Controller Comparison

All controller implementations should be evaluated using consistent test conditions.

### 9.1 Standard Test Conditions

For a meaningful comparison, use the same:

- Mechanical Plant reference
- Electromechanical Model reference
- Motor parameters
- Initial joint positions and velocities
- Simulation duration
- Solver configuration
- Actuator constraints, if implemented

Only the controller design should change between equivalent comparison experiments.

### 9.2 Required Measurements

Record:

| Signal | Description |
|---|---|
| `theta0` | Rotary-arm angular position |
| `dtheta0` | Rotary-arm angular velocity |
| `theta1` | Pendulum angular position |
| `dtheta1` | Pendulum angular velocity |
| `Voltage` | Controller motor-voltage command |

The motor-voltage command must be measured at the controller output before entering the Electromechanical subsystem.

### 9.3 Performance Evaluation

The primary stabilization objective is:

\[
x(t)\rightarrow
\begin{bmatrix}
0&0&0&0
\end{bmatrix}^{T}
\]

Performance comparisons should consider:

- Pendulum settling time
- Pendulum overshoot and oscillation
- Rotary-arm displacement
- Steady-state errors
- Angular-velocity response
- Peak motor-voltage demand
- Stability under the selected initial disturbance

Stable closed-loop eigenvalues do not independently establish global nonlinear stability or hardware feasibility.

## 10. Referenced Subsystem Maintenance

All shared components are maintained as Simulink Referenced Subsystems.

| Component | Source File |
|---|---|
| Reference Mechanical Plant | `Plant.slx` |
| Electromechanical Model | `Electromechanical.slx` |
| Pole-Placement Controller | `Pole_Placement.slx` |
| Future Controllers | Controller-specific referenced files |

Editing and saving a referenced subsystem updates its shared implementation. Dependent simulations use the updated version when their references are refreshed or models are updated.

### Development Rules

1. Preserve the shared subsystem interfaces when introducing new controllers.
2. Do not duplicate the internal mechanical or motor blocks across simulations.
3. Do not modify shared mechanical parameters to improve the performance of an individual controller.
4. Keep controller gains associated with their corresponding plant model.
5. Avoid duplicate referenced subsystem filenames on MATLAB's search path.
6. Verify dependent simulations after modifying shared component files.
7. Repeat linearization when changes to plant dynamics invalidate the saved state-space model.

Referenced subsystem files must remain accessible through MATLAB's search path.

Workspace variables used inside referenced subsystems must be initialized before compilation or simulation.

## 11. Troubleshooting

### 11.1 Missing Motor Parameters

**Symptom:** Simulink reports that `motor.Ke`, `motor.Kt`, or `motor.Rm` is undefined.

**Cause:** The motor initialization has not been executed in the current MATLAB session.

**Resolution:** Load the saved linearization and run:

```matlab
analyze_simscape_linearization;
```

This initializes the motor parameter structure used by the Electromechanical subsystem.

### 11.2 Undefined Controller Gain

**Symptom:** Simulink reports that `K_pp_sim` is undefined.

**Cause:** The pole-placement design script has not been executed.

**Resolution:** After generating `A` and `B`, run:

```matlab
design_simscape_pole_placement;
```

The script generates the feedback gain required by `Pole_Placement.slx`.

### 11.3 Missing Referenced Subsystem

**Symptom:** Simulink cannot locate `Plant.slx`, `Electromechanical.slx`, or another referenced subsystem.

**Cause:** The corresponding directory is missing from MATLAB's path, or the file has been moved or renamed.

**Resolution:** Add the subsystem directory:

```matlab
addpath(fullfile(pwd, 'Modeling', 'Subsystems'));
```

Verify that MATLAB resolves the intended source:

```matlab
which('Plant.slx')
which('Electromechanical.slx')
which('Pole_Placement.slx')
```

The returned paths should point to `Modeling/Subsystems/`.

### 11.4 Missing URDF Geometry

**Symptom:** The imported assembly appears incomplete or mesh files cannot be located.

**Cause:** The imported model cannot resolve one or more mesh assets.

**Resolution:** Verify that the URDF and its referenced mesh files exist in their expected locations. Inspect the geometry-file paths associated with the imported Simscape solids.

Importing a new URDF may require regenerating the mechanical model with valid geometry references.

### 11.5 Saved Linearization Is Outdated

**Symptom:** A controller was designed using state-space matrices that do not match the current mechanical or electromechanical model.

**Cause:** The plant parameters or system dynamics changed after `linsys1.mat` was generated.

**Resolution:** Repeat Model Linearizer analysis, save the updated model under the appropriate versioned filename, and recalculate controller gains.

Do not reuse previous gains without checking closed-loop stability.

### 11.6 Simulation Does Not Stabilize

Possible causes include:

- Incorrect state ordering
- Incorrect feedback sign
- Inconsistent joint-angle convention
- Missing controller gain initialization
- Gains designed for a different plant
- Excessive initial disturbance
- Incorrect motor or mechanical parameters
- Actuator limits or nonlinear effects not represented during linear design

Check the four measured states, controller voltage, and linear closed-loop eigenvalues before changing controller gains.

## 12. Implementation Status and Limitations

### Implemented

- Imported Furuta pendulum mechanical model in Simscape Multibody
- DC motor electromechanical subsystem
- Open-loop simulation
- Four-state sensing
- Upright state-space linearization
- Open-loop eigenvalue analysis
- Controllability and observability analysis
- Pole-placement controller
- Linear closed-loop eigenvalue verification
- Nonlinear Simscape stabilization testing
- Reusable referenced subsystem architecture

### Current Limitations

- The mechanical simulation uses a reference URDF rather than the final project CAD assembly.
- The motor parameters are preliminary assumptions.
- Actuator voltage, current, and driver limits have not been fully validated.
- The simplified motor model neglects armature inductance.
- Stability verification is local to the linearization operating point.
- Controller robustness and physical hardware performance require further validation.
- The proposed `Plant_Main` and `linsys2` migration workflow has not yet been implemented.

The current Simscape implementation provides the shared simulation environment for continued controller development. Future control strategies and mechanical revisions must preserve the documented component interfaces and undergo independent verification before integration.