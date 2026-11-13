addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
addpath('./tpik');
clc; clear; close all;

dt       = 0.005;
endTime  = 50;

robotModel = UvmsModel();
robotModel.eta = [48.5; 11.5; -33; deg2rad(0); deg2rad(0); deg2rad(0)];
robotModel.updateTransformations();

sim = UvmsSim(dt, robotModel, endTime);
unity = UnityInterface("127.0.0.1");

% Inizializzazione task
task_position         = TaskVehiclePosition();
task_misalignment     = TaskVehicleMisalignment();
task_altitude_control = TaskVehicleAltitudeControl();
task_altitude_min     = TaskVehicleAltitudeMin();
task_orientation      = TaskVehicleOrientation();

% Esercizio 4: Azioni e lista unificata
safe_navigation_tasks = {task_altitude_min, task_misalignment, task_position, task_orientation};
landing_tasks         = {task_altitude_control, task_misalignment, task_position};
unified_list          = {task_altitude_min, task_altitude_control, task_misalignment, task_position, task_orientation};

actionManager = ActionManager(robotModel);
actionManager.dt = dt;
actionManager.addAction("safe_navigation", safe_navigation_tasks);
actionManager.addAction("landing", landing_tasks);
actionManager.setUnifyingTaskList(unified_list);

actionManager.setCurrentAction("safe_navigation");
task_set = unified_list; 

w_arm_goal_position = [12.2025, 37.3748, -39.8860]';
w_arm_goal_orientation = [0, pi, pi/2];
w_vehicle_goal_position = [50; -12.5; -33]; 
w_vehicle_goal_orientation = [deg2rad(0), deg2rad(0), deg2rad(0)];

robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, task_set);

tic
for step = 1:sim.maxSteps
    robotModel.altitude = unity.receiveAltitude(robotModel);

    % Transizione ad atterraggio
    xy_error = norm(robotModel.vehicleGoalPosition(1:2) - robotModel.eta(1:2));
    if xy_error < 0.5 && actionManager.currentActionName == "safe_navigation"
        disp('--- Waypoint Raggiunto: Inizio atterraggio ---');
        actionManager.setCurrentAction("landing");
    end

    [v_nu, q_dot] = actionManager.computeICAT(robotModel);
    sim.step(v_nu, q_dot);
    unity.send(robotModel);
    logger.update(sim.time, sim.loopCounter);

    if mod(sim.loopCounter, round(1 / sim.dt)) == 0
        z_v = robotModel.wTv(1:3, 3);
        z_w = [0; 0; 1];

        error = norm(cross(z_v, z_w));
        fprintf('t = %.2f s | Azione: %s | alt = %.2f m\n', sim.time, actionManager.currentActionName, robotModel.altitude);
        fprintf('misalignment = %.2f', error);
    end

    SlowdownToRealtime(dt);
end

logger.plotAll();
delete(unity);