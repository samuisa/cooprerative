classdef UvmsModel < handle
    %UVMSMODEL Kinematic state and geometry of the UVMS.
    %
    % This handle class is the shared state passed to the controller, simulator, logger and Unity interface. It stores the manipulator configuration, the vehicle pose, forward-kinematic transforms and the desired tool/vehicle poses.
    %
    % Transform notation follows aTb = ^a T_b: it maps coordinates expressed in frame b into frame a. The relevant frames are
    %
    %   w: inertial/world frame      v: vehicle body frame
    %   b: manipulator base frame    e: arm end-effector frame
    %   t: tool frame                g: goal frame
    %
    % Vehicle orientation is stored as roll, pitch and yaw and converted to Rz(yaw)*Ry(pitch)*Rx(roll). Vehicle linear and angular velocities are instead expressed in the vehicle frame; angular velocity [p q r] is not the time derivative of the Euler-angle vector.
    %
    % Example:
    %   model = UvmsModel('Robust');
    %   model.updateTransformations();

    properties
        %% --- State variables ---
        q           % [7x1] manipulator joint positions [rad].
        q_dot       % [7x1] manipulator joint velocities [rad/s].
        eta         % [6x1] vehicle pose [x y z roll pitch yaw]' in world.
        v_nu        % [6x1] vehicle twist [vx vy vz p q r]' in vehicle frame.
        altitude    % Scalar altitude measurement supplied by Unity [m].

        %% --- Geometry ---
        vTb         % [4x4] fixed pose of manipulator base with respect to vehicle.
        eTt         % [4x4] fixed pose of tool with respect to end-effector.

        %% --- Limits ---
        jlmin       % [7x1] manipulator lower joint limits [rad].
        jlmax       % [7x1] manipulator upper joint limits [rad].
        q_dot_max   % [7x1] maximum absolute manipulator joint velocities [rad/s].
        v_nu_max    % [6x1] maximum absolute vehicle twist [vx vy vz p q r]' [m/s; rad/s].

        %% --- Transformations ---
        wTv         % [4x4] vehicle pose with respect to world.
        vTw         % [4x4] world pose with respect to vehicle (inverse of wTv).
        vTe         % [4x4] end-effector pose with respect to vehicle.
        vTt         % [4x4] tool pose with respect to vehicle.
        wTt         % [4x4] tool pose with respect to world.
        wTg         % [4x4] desired tool pose with respect to world.
        wTgv        % [4x4] desired vehicle pose with respect to world.
        bTe         % [4x4] end-effector pose with respect to arm base.
        vTg         % [4x4] desired tool pose with respect to vehicle.

        %% --- Goals ---
        wRg                % [3x3] desired tool orientation in world.
        goalPosition       % [3x1] desired tool position in world [m].
        wRgv               % [3x3] desired vehicle orientation in world.
        vehicleGoalPosition% [3x1] desired vehicle position in world [m].
    end

    methods
        function obj = UvmsModel(robotType)
            %UVMSMODEL Construct a model with a predefined arm mounting.
            %
            % Optional input:
            %   robotType - Character vector or string selecting 'DexROV' or 'Robust'. If omitted, 'Robust' is used.
            %
            % Output:
            %   obj - Initialized UvmsModel handle object.
            %
            % The constructor initializes geometry, a default configuration and identity goal transforms, then computes forward kinematics.

            if nargin < 1
                robotType = 'Robust';
            end

            % vTb is fixed: changing the vehicle pose or arm joints does not alter the rigid mounting between the two subsystems.
            switch robotType
                case 'DexROV'
                    obj.vTb = [RPYToRotationMatrix(pi, 0, pi) [0.167; 0; -0.43]; 0 0 0 1];
                otherwise % 'Robust'
                    obj.vTb = [RPYToRotationMatrix(0, 0, pi) [0.85; 0; -0.42]; 0 0 0 1];
            end

            % Initial arm configuration and vehicle pose used by the exercise.
            obj.q      = [-0.0031 0 0.0128 -1.2460 0.0137 0.0853 -pi/2]';
            obj.eta    = [8.5 38.5 -38 0 -0.06 0.5]';

            % Initialize velocity and sensor state before they are read by the ActionManager, logger or asynchronous Unity interface.
            obj.q_dot    = zeros(size(obj.q));
            obj.v_nu     = zeros(6,1);
            obj.altitude = NaN; % No altitude measurement has been received yet.

            % Mechanical joint limits used by joint-limit avoidance tasks.
            obj.jlmin  = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
            obj.jlmax  = [ 2.9; 1.65; 2.9; 0.01; 2.9; 1.25; 2.8];

            % Symmetric velocity limits used by the saturated iCAT recursion. ActionManager obtains the corresponding lower limits by changing their sign.
            obj.q_dot_max = 2*ones(7,1);
            obj.v_nu_max = [1; 1; 1; 0.3; 0.3; 0.3];

            % Initialize every transform before the first kinematic update.
            obj.wTv = eye(4);
            obj.vTw = eye(4);
            obj.vTe = eye(4);
            obj.vTt = eye(4);
            obj.wTt = eye(4);
            obj.eTt = eye(4); % tool coincides with end-effector by default

            % Identity goals are placeholders until setGoal is called.
            obj.wTg = eye(4);
            obj.wTgv = eye(4);

            updateTransformations(obj);
        end

        function setGoal(obj, toolPosition, toolOrientation, vehiclePosition, vehicleOrientation)
            %SETGOAL Store desired tool and vehicle poses in the world frame.
            %
            % Inputs:
            %   toolPosition       - [3x1] desired tool position in world [m].
            %   toolOrientation    - [3x1] desired [roll pitch yaw] [rad].
            %   vehiclePosition    - [3x1] desired vehicle position in world [m].
            %   vehicleOrientation - [3x1] desired [roll pitch yaw] [rad].
            %
            % Output:
            %   None. Updates the goal properties of this handle object.

            % Build homogeneous goal transforms from RPY orientation and Cartesian position. RPYToRotationMatrix implements Rz(yaw)*Ry(pitch)*Rx(roll).
            obj.wRg = RPYToRotationMatrix(toolOrientation(1), toolOrientation(2), toolOrientation(3));
            obj.goalPosition = toolPosition;
            obj.wTg = [obj.wRg obj.goalPosition; 0 0 0 1];

            obj.wRgv = RPYToRotationMatrix(vehicleOrientation(1), vehicleOrientation(2), vehicleOrientation(3));
            obj.vehicleGoalPosition = vehiclePosition;
            obj.wTgv = [obj.wRgv obj.vehicleGoalPosition; 0 0 0 1];
        end

        function updateTransformations(obj)
            %UPDATETRANSFORMATIONS Recompute UVMS forward kinematics.
            %
            % Input:
            %   None beyond the current obj.q, obj.eta and fixed geometry.
            %
            % Output:
            %   None. Updates wTv, vTw, vTg, bTe, vTe, vTt and wTt.

            % Convert the vehicle world pose from [position; RPY] to a homogeneous transform and compute the inverse frame mapping.
            obj.wTv = [RPYToRotationMatrix(obj.eta(4), obj.eta(5), obj.eta(6)) obj.eta(1:3); 0 0 0 1];
            obj.vTw = inv(obj.wTv);

            % Express the fixed world-frame tool goal in vehicle coordinates; this quantity changes whenever the vehicle moves.
            obj.vTg = obj.vTw * obj.wTg;

            % Propagate the arm forward kinematics along the frame chain b -> v and the fixed end-effector-to-tool displacement.
            obj.bTe = RobustEndEffectorTransform(obj.q);
            obj.vTe = obj.vTb * obj.bTe;
            obj.vTt = obj.vTe * obj.eTt;
            obj.wTt = obj.wTv * obj.vTt;
        end
    end
end
