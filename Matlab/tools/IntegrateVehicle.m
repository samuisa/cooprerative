function etaNext = IntegrateVehicle(eta, v_nu, dt)
%INTEGRATEVEHICLE Integrate the vehicle pose from its body-frame twist.
%
% Inputs:
%   eta  - [6x1] vehicle pose [x y z roll pitch yaw]' with position expressed in the world frame and orientation represented by RPY angles [m; rad].
%   v_nu - [6x1] vehicle twist [vx vy vz omega_x omega_y omega_z]' expressed in the vehicle frame [m/s; rad/s].
%   dt   - Positive integration interval [s].
%
% Output:
%   etaNext - [6x1] vehicle pose after one explicit Euler integration step.

% Rotate the body-frame linear velocity into the world frame.
wRv = RPYToRotationMatrix(eta(4), eta(5), eta(6));
wLinearVelocity = wRv*v_nu(1:3);

% Convert body angular velocity to derivatives of the stored Euler angles.
rpyDot = BodyAngularVelocityToRPYRates(eta(4:6), v_nu(4:6));

% Integrate the derivative of the pose coordinates with an explicit Euler step.
etaDot = [wLinearVelocity; rpyDot];
etaNext = eta + etaDot*dt;
end
