# UVMS task-priority exercise template

This folder contains the MATLAB skeleton used to study kinematic control of an underwater vehicle-manipulator system (UVMS). The code follows the control organization introduced in the course notes and in:

- E. Simetti and G. Casalino, *A Novel Practical Technique to Integrate Inequality Control Objectives and Task Transitions in Priority Based Control*, 2016.
- E. Simetti et al., *Task priority control of underwater intervention systems: theory and applications*, 2018.

The template implements the Kinematic Control Layer. It computes velocity references from a prioritized list of tasks and integrates them with a simple kinematic simulator. On a real UVMS, a lower Dynamic Control Layer would track the same velocity references.

## Architecture at a glance

`RobustMain.m` connects the following blocks:

1. `UvmsModel` stores robot state, goals and frame transformations.
2. Each subclass of `Task` computes a reference rate `xdotbar`, a Jacobian `J` and an activation matrix `A`.
3. `ActionManager` updates the tasks and calls the saturated iCAT solver from highest to lowest priority.
4. `UvmsSim` integrates the resulting arm and vehicle velocities.
5. `SimulationLogger` records state and task signals.
6. `UnityInterface` exchanges state and altitude data with the Unity viewer.

An **action** is a cell array of tasks. The first element has the highest priority. Lower-priority tasks may use only motion that does not interfere with the tasks above them. Actions are registered and selected using descriptive string names:

```matlab
actionManager = ActionManager(robotModel);
actionManager.addAction("move_tool", {task_tool});
actionManager.setCurrentAction("move_tool");
```

## Central notation

The complete system velocity is

```matlab
ydot = [q_dot; v_nu];
```

where:

- `q_dot` is the 7-by-1 manipulator joint velocity;
- `v_nu = [vx vy vz p q r]'` is the 6-by-1 vehicle twist expressed in the vehicle frame;
- `ydot` is therefore 13-by-1.

For a task of dimension `m`:

```matlab
xdot = J * ydot;
```

- `xdotbar` is the `m`-by-1 desired task-space velocity;
- `J` is the `m`-by-13 task Jacobian;
- `A` is an `m`-by-`m` diagonal activation matrix.

An activation value of zero leaves the associated degree of freedom available to lower priorities. A value of one makes the component fully active. Smooth values between zero and one are used for inequality objectives and task transitions.

Each task also stores its inverse-regularization parameters. `lambda` requests the maximum damping applied to weak singular directions, `threshold` defines the singular-value interval in which damping is introduced, and `gain` weights the protection of directions already constrained by higher-priority tasks. Their default values reproduce the original solver settings, but keeping them inside the task allows different objectives to use different conditioning.

System-velocity saturation is enforced inside the iCAT priority recursion rather than after the final solution has been computed. `UvmsModel` stores the positive limits `q_dot_max = 2 rad/s` for every arm joint and `v_nu_max = [1 1 1 0.3 0.3 0.3]'` for the vehicle twist; `ActionManager` builds the corresponding symmetric lower limits. After each task, iCAT returns the velocity range still available to lower priorities, preventing a lower-priority contribution from exceeding the limits established for `[q_dot; v_nu]`. This mechanism is distinct from the task-space bound applied to `xdotbar` in `TaskTool`.

Transform names follow `aTb = ^a T_b`: the transform maps coordinates written in frame `b` into frame `a`. The frame letters used in the code are:

- `w`: inertial/world;
- `v`: vehicle body;
- `b`: manipulator base;
- `e`: manipulator end-effector;
- `t`: tool;
- `g`: goal.

Vehicle pose uses `[x y z roll pitch yaw]'`. The angular part of `v_nu` is expressed in the vehicle frame and is not equal to the derivative of the Euler-angle vector; `BodyAngularVelocityToRPYRates.m` performs the required conversion.

## Adding a task

Create a subclass of `Task` and implement these three updates:

1. `updateReference`: compute the feedback or externally commanded reference rate and apply any required velocity bounds.
2. `updateJacobian`: map `[q_dot; v_nu]` to the chosen task-space rate. Check dimensions, row ordering and reference frames carefully.
3. `updateActivation`: build a diagonal matrix with entries in `[0, 1]`.

Then instantiate the task in `RobustMain.m` and insert it into `task_set` at the desired priority. Safety and physical-constraint objectives normally precede the action objective; optimization or preferred-shape tasks normally follow it.

## Running the exercise

Set the MATLAB current folder to this template directory and run:

```matlab
RobustMain
```

The script expects the helper folders referenced by its `addpath` calls (`tools`, `icat`, `robust_robot` and `tpik`) and a compatible Unity UDP endpoint when visualization and altitude feedback are enabled.

`SlowdownToRealtime` uses MATLAB's shared `tic`/`toc` stopwatch. Initialize the stopwatch with `tic` before its first call if real-time pacing is enabled.

The expanded files `RobustEndEffectorTransform.m` and `RobustJacobian.m` are generated analytic expressions. Students normally use these functions as provided rather than editing individual formula lines.
