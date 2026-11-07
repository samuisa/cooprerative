classdef TaskTool < Task
    %TASKTOOL Six-dimensional Cartesian pose task for the UVMS tool.
    %
    % The task drives the tool frame vTt toward the goal frame vTg. Both frames are expressed relative to the vehicle, so the resulting error, reference velocity and Jacobian use a consistent vehicle-frame representation. All six components are permanently active.

    properties
        feedbackGain = 0.2        % Proportional gain that converts Cartesian error into a velocity reference [1/s].
        maxAngularVelocity = 0.2  % Maximum absolute component of the angular velocity reference [rad/s].
        maxLinearVelocity = 0.2   % Maximum absolute component of the linear velocity reference [m/s].
    end

    methods
        function updateReference(obj, robot)
            %UPDATEREFERENCE Build a bounded Cartesian feedback velocity.
            %
            % Input:
            %   robot - UvmsModel with current tool transform robot.vTt and desired tool transform robot.vTg.
            %
            % Output:
            %   None. Updates obj.xdotbar as [angular; linear] velocity, expressed in the vehicle frame.

            % CartesianError returns geometric errors expressed in the vehicle frame. The proportional gain converts them into angular and linear velocity references.
            [angularError, linearError] = CartesianError(robot.vTg, robot.vTt);
            obj.xdotbar = obj.feedbackGain*[angularError; linearError];

            % Limit angular and linear parts independently while preserving the direction within each three-dimensional subvector.
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), obj.maxAngularVelocity);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), obj.maxLinearVelocity);
        end

        function updateJacobian(obj, robot)
            %UPDATEJACOBIAN Build the tool Jacobian for arm and vehicle motion.
            %
            % Input:
            %   robot - UvmsModel providing q and the transforms vTb, vTe, vTt and eTt.
            %
            % Output:
            %   None. Updates obj.J, a [6x13] matrix with columns ordered as [seven arm joints, six vehicle velocities]. Rows follow the [angular; linear] convention used by obj.xdotbar.

            % The generated arm Jacobian refers to the end-effector and is expressed in the arm-base frame.
            bJe = RobustJacobian(robot.q);

            % Shift the Jacobian from the end-effector origin to the tool origin (Ste), then rotate it from arm-base to vehicle axes.
            Ste = [eye(3) zeros(3);  -skew(robot.vTe(1:3,1:3)*robot.eTt(1:3,4)) eye(3)];
            Jt_a  = Ste * [robot.vTb(1:3,1:3) zeros(3,3); zeros(3,3) robot.vTb(1:3,1:3)] * bJe;

            % Vehicle motion affects the tool both directly and through the lever arm from the vehicle origin to the tool position.
            Jt_v = [zeros(3) eye(3); eye(3) -skew(robot.vTt(1:3,4))];

            % Concatenate according to ydot = [q_dot; v_nu].
            obj.J = [Jt_a Jt_v];
        end

        function updateActivation(obj, robot)
            %UPDATEACTIVATION Keep all Cartesian components fully active.
            %
            % Input:
            %   robot - UvmsModel (unused; retained for the common Task API).
            %
            % Output:
            %   None. Sets obj.A to the [6x6] identity matrix.
            %#ok<INUSD>
            obj.A = eye(6);
        end
    end
end
