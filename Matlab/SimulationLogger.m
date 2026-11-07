classdef SimulationLogger < handle
    %SIMULATIONLOGGER Preallocate, store and plot UVMS simulation signals.
    %
    % One column is reserved for each simulation sample. In addition to the robot state, the logger stores the diagonal of every task activation matrix and the complete task reference-rate vector at every iteration.

    properties
        t            % [1 x N] simulation time samples [s].
        q            % [7 x N] arm joint positions [rad].
        q_dot        % [7 x N] arm joint velocities [rad/s].
        eta          % [6 x N] vehicle pose [x y z roll pitch yaw]'.
        v_nu         % [6 x N] vehicle body-frame twists [vx vy vz p q r]'.
        a            % [M x N x T] diagonals of task activation matrices.
        xdotbar_task % [T x N] cell array of task reference-rate vectors.
        robot        % UvmsModel handle read during logging.
        task_set     % Cell array of Task handles, in priority order.
    end

    methods
        function obj = SimulationLogger(maxLoops, robotModel, task_set)
            %SIMULATIONLOGGER Allocate storage for a complete simulation.
            %
            % Inputs:
            %   maxLoops  - Maximum number of samples to store.
            %   robotModel - UvmsModel handle whose state will be logged.
            %   task_set  - Cell array of Task objects to monitor.
            %
            % Output:
            %   obj - Initialized SimulationLogger handle object.
            obj.robot = robotModel;
            obj.task_set = task_set;

            obj.t = zeros(1, maxLoops);
            obj.q = zeros(7, maxLoops);
            obj.q_dot = zeros(7, maxLoops);
            obj.eta = zeros(6, maxLoops);
            obj.v_nu = zeros(6, maxLoops);

            % Tasks may have different dimensions. Allocate the first axis to the largest task and leave unused entries equal to zero.
            maxDiagSize = max(cellfun(@(t) size(t.A,1), task_set));
            obj.a = zeros(maxDiagSize, maxLoops, length(task_set));

            % A cell array is required because task reference vectors may have different lengths.
            obj.xdotbar_task = cell(length(task_set), maxLoops);
        end

        function update(obj, t, loop)
            %UPDATE Store one sample of robot and task data.
            %
            % Inputs:
            %   t    - Current simulation time [s].
            %   loop - One-based destination column in the preallocated arrays.
            %
            % Output:
            %   None. Writes into the logger arrays in place.

            % Copy the current state from the shared UvmsModel handle.
            obj.t(loop) = t;
            obj.q(:, loop) = obj.robot.q;
            obj.q_dot(:, loop) = obj.robot.q_dot;
            obj.eta(:, loop) = obj.robot.eta;
            obj.v_nu(:, loop) = obj.robot.v_nu;

            % Only the activation diagonal is required because task activation matrices are diagonal by construction in the iCAT formulation.
            for i = 1:length(obj.task_set)
                diagA = diag(obj.task_set{i}.A);           % extract diagonal
                obj.a(1:length(diagA), loop, i) = diagA;
                obj.xdotbar_task{i, loop} = obj.task_set{i}.xdotbar;
            end
        end

        function plotAll(obj)
            %PLOTALL Plot the principal state and task-activation histories.
            %
            % Input/output:
            %   None. Creates MATLAB figures from the stored arrays.

            % Figure 1: manipulator generalized coordinates and velocities.
            figure(1);
            subplot(2,1,1);
            plot(obj.t, obj.q, 'LineWidth', 1);
            legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
            subplot(2,1,2);
            plot(obj.t, obj.q_dot, 'LineWidth', 1);
            legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6','qd_7');

            % Figure 2: vehicle world pose and body-frame twist.
            figure(2);
            subplot(2,1,1);
            plot(obj.t, obj.eta, 'LineWidth', 1);
            legend('x','y','z','roll','pitch','yaw');
            subplot(2,1,2);
            plot(obj.t, obj.v_nu, 'LineWidth', 1);
            legend('xdot','ydot','zdot','omega_x','omega_y','omega_z');

            % Figure 3: one subplot per task. Each curve is one diagonal activation component and therefore remains between zero and one.
            figure(3);
            for i = 1:size(obj.a,3)
                subplot(size(obj.a,3),1,i);
                plot(obj.t, squeeze(obj.a(:, :, i))', 'LineWidth', 1);
                title(['Task ', num2str(i), ' Activations (diagonal)']);
            end
        end
    end
end
