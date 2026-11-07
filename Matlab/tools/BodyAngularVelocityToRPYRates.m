function rpyDot = BodyAngularVelocityToRPYRates(rpy, bodyAngularVelocity)
%BODYANGULARVELOCITYTORPYRATES Convert body angular velocity to Euler-angle rates.
%
% Inputs:
%   rpy                 - [3x1] or [1x3] current [roll pitch yaw] angles [rad].
%   bodyAngularVelocity - [3x1] or [1x3] angular velocity expressed in the body frame [rad/s].
%
% Output:
%   rpyDot - [3x1] derivatives [roll_dot pitch_dot yaw_dot]' [rad/s].
%
% The mapping corresponds to R = Rz(yaw)*Ry(pitch)*Rx(roll). It is singular when cos(pitch) = 0, so trajectories should stay away from pitch = +/-pi/2.

omegaX = bodyAngularVelocity(1);
omegaY = bodyAngularVelocity(2);
omegaZ = bodyAngularVelocity(3);

tanPitch = tan(rpy(2));
sinRoll = sin(rpy(1));
cosRoll = cos(rpy(1));
cosPitch = cos(rpy(2));

rpyDot = [omegaX + tanPitch*sinRoll*omegaY + tanPitch*cosRoll*omegaZ; cosRoll*omegaY - sinRoll*omegaZ; sinRoll/cosPitch*omegaY + cosRoll/cosPitch*omegaZ];
end
