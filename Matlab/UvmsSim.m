classdef UvmsSim < handle
    %UVMSSIM Kinematic simulator for the vehicle-manipulator system.
    %
    % The simulator integrates the arm joint rates and the vehicle body-frame twist with a fixed-step explicit Euler scheme. After every step it asks UvmsModel to recompute the frame transformations. Control, logging and visualization are intentionally handled by separate classes.

    properties
        robot       % UvmsModel handle containing the simulated state.
        dt          % Positive integration time step [s].
        time        % Current simulation time [s].
        maxSteps    % Number of iterations required to reach endTime.
        loopCounter % One-based sample index used by the logger.
    end

    methods
        function obj = UvmsSim(dt, robotModel, endTime)
            %UVMSSIM Construct a fixed-step kinematic simulator.
            %
            % Inputs:
            %   dt         - Positive integration time step [s].
            %   robotModel - UvmsModel handle to update in place.
            %   endTime    - Requested simulation duration [s].
            %
            % Output:
            %   obj - Initialized UvmsSim handle object.
            obj.dt = dt;
            obj.robot = robotModel;
            obj.time = 0;
            obj.loopCounter = 1;
            obj.maxSteps = ceil(endTime/dt);
        end

        function step(obj, v_nu, q_dot)
            %STEP Advance the complete UVMS state by one sample.
            %
            % Inputs:
            %   v_nu - [6x1] vehicle twist [vx vy vz p q r]' expressed in the vehicle frame.
            %   q_dot - [7x1] arm joint velocity [rad/s].
            %
            % Output:
            %   None. Updates obj.robot, obj.time and obj.loopCounter.

            % Store the applied velocities so they are available to the logger and any task that uses the current commanded motion.
            obj.robot.q_dot = q_dot;
            obj.robot.v_nu = v_nu;

            % IntegrateVehicle rotates translational velocity into the world frame and maps body angular velocity to Euler-angle rates.
            obj.robot.eta = IntegrateVehicle(obj.robot.eta, v_nu, obj.dt);

            % Joint coordinates can be integrated directly because q_dot is expressed in the corresponding generalized coordinates.
            obj.robot.q = obj.robot.q + q_dot * obj.dt;

            % All downstream tasks must see transforms consistent with the new vehicle and manipulator configuration.
            obj.robot.updateTransformations();

            % Advance simulation time and the one-based logging index.
            obj.time = obj.time + obj.dt;
            obj.loopCounter = obj.loopCounter + 1;
        end
    end
end
