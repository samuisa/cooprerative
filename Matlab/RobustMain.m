%% ROBUST UVMS TASK-PRIORITY EXERCISE
% This script assembles the main blocks of the control architecture:
%
%   task definitions -> ActionManager/iCAT -> kinematic simulator -> Unity
%
% At every iteration, the Kinematic Control Layer computes the system velocity ydot = [q_dot; v_nu].
% UvmsSim integrates this reference directly; in a real robot, a Dynamic Control Layer would track the same reference.
%
% Run this script with the current MATLAB folder set to the template root so that the relative paths below resolve consistently.

%% Make exercise dependencies visible to MATLAB
addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
addpath('./tpik');
clc; clear; close all;

%% Configure and initialize the simulation
% dt is both the integration step and the nominal controller sample time.
dt       = 0.005;
endTime  = 50;

% UvmsModel owns state and kinematics; UvmsSim updates that shared handle.
robotModel = UvmsModel();

robotModel.eta = [10.5; 35.5; -20; -pi/3; pi/3; pi/2];
robotModel.updateTransformations();

sim = UvmsSim(dt, robotModel, endTime);

% Unity is used only as viewer/sensor endpoint; it is not the simulator.
unity = UnityInterface("127.0.0.1");

%% Define one action as a strict-priority stack of tasks
% The first cell is the highest-priority task. Add safety tasks before the action objective when extending this exercise (for example, joint limits).
% task_tool = TaskTool();
% move_tool_tasks = {task_tool};

% Actions are registered and selected by name; the task order still defines strict priority within each action.
% actionManager = ActionManager(robotModel);
% actionManager.addAction("move_tool", move_tool_tasks);
% actionManager.setCurrentAction("move_tool");

% The logger monitors the tasks belonging to the action selected above.

task_position       = TaskVehiclePosition();        % exercise 1
task_misalignment   = TaskVehicleMisalignment();    % exercise 2
task_altitude       = TaskVehicleAltitude();
task_orientation    = TaskVehicleOrientation();

vehicle_tasks = {task_altitude, task_orientation, task_position, task_misalignment}; %task priorities

actionManager = ActionManager(robotModel);
actionManager.addAction("move_vehicle", vehicle_tasks);
actionManager.setCurrentAction("move_vehicle");

task_set = vehicle_tasks; % task set to be considered

%% Set desired poses
% Positions are in metres and orientations are [roll pitch yaw] in radians. Both poses are expressed in the inertial/world frame.
w_arm_goal_position = [12.2025, 37.3748, -39.8860]';
w_arm_goal_orientation = [0, pi, pi/2];
w_vehicle_goal_position = [10.5; 37.5; -45];
w_vehicle_goal_orientation = [0, 0, 0];

% Store the goals as homogeneous transforms in the shared model.
robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

% Preallocate one column for the initial state plus all integration steps.
logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, task_set);

%% Run the closed-loop kinematic simulation
tic
for step = 1:sim.maxSteps
    % Read the newest sensor sample. If none is available, the interface keeps the previous altitude value.
    robotModel.altitude = unity.receiveAltitude(robotModel);

    % Update every task and solve the current strict-priority hierarchy.
    [v_nu, q_dot] = actionManager.computeICAT(robotModel);

    % Integrate arm and vehicle velocities, then refresh all transforms.
    sim.step(v_nu, q_dot);

    % Send the state after integration so the viewer displays the new sample.
    unity.send(robotModel);

    % Store state, task references and activations at the current sample.
    logger.update(sim.time, sim.loopCounter);

    misalignment_error_norm = norm(cross(robotModel.wTv(1:3,3), [0;0;1]));

    % Print a compact status update approximately once per simulated second.
    if mod(sim.loopCounter, round(1 / sim.dt)) == 0
        fprintf('t = %.2f s\n', sim.time);
        fprintf('alt = %.2f m\n', robotModel.altitude);
        fprintf('misalignment = %.2f m\n', misalignment_error_norm);
        fprintf('abs position = [%.2f; %.2f; %.2f] \n', robotModel.wTv(1,4), robotModel.wTv(2,4), robotModel.wTv(3,4));
        fprintf('goal position = [%.2f; %.2f; %.2f] \n', robotModel.vehicleGoalPosition(1), robotModel.vehicleGoalPosition(2), robotModel.vehicleGoalPosition(3));
        fprintf('altitude = %.2f \n', robotModel.eta(2));
        
        fprintf('vehicle orientation = \n');
        disp(robotModel.wTv(1:3, 1:3));
    end

    % Busy-wait until one sample period has elapsed when real-time playback is desired. 
    % Remove this call to execute the simulation as fast as possible.
    SlowdownToRealtime(dt);
end

%% Review results and release communication resources
logger.plotAll();

% Explicit deletion closes UDP sockets before the script terminates.
delete(unity);
