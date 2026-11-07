classdef Task < handle
    %TASK Abstract interface implemented by every task-priority objective.
    %
    % A concrete task maps the UVMS system velocity ydot to a task-space velocity through
    %
    %   xdot = J * ydot,       ydot = [q_dot; v_nu].
    %
    % Before each task-priority inversion, the task must update its desired reference rate xdotbar, Jacobian J and diagonal activation matrix A.
    % Activation values vary from 0 (inactive) to 1 (fully active).
    %
    % Each task also owns the parameters used by the regularized inverse:
    %   - lambda requests the maximum damping for weak singular directions;
    %   - threshold defines the singular-value region where damping is active. Values above the threshold are left unregularized, while smaller values receive a smoothly increasing damping contribution;
    %   - gain weights the penalty on control directions already constrained by higher-priority tasks. A larger value reinforces priority preservation, but may also make the inverse more conservative.

    properties
        xdotbar = [] % [m x 1] desired task-space velocity/reference rate.
        J = []       % [m x n] task Jacobian for ydot = [q_dot; v_nu].
        A = []       % [m x m] diagonal task activation matrix, entries in [0,1].

        % Singular-value-oriented regularization parameters. The defaults reproduce the values originally used directly by ActionManager.
        lambda = 1e-4
        threshold = 0.01
        gain = 10
    end

    methods (Abstract)
        %UPDATEREFERENCE Update xdotbar from the current robot state/goal.
        updateReference(obj, robot)

        %UPDATEJACOBIAN Update the mapping from system to task velocity.
        updateJacobian(obj, robot)

        %UPDATEACTIVATION Update the task activation matrix A.
        updateActivation(obj, robot)
    end
end


