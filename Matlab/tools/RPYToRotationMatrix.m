function rotationMatrix = RPYToRotationMatrix(roll, pitch, yaw)
%RPYTOROTATIONMATRIX Convert roll, pitch and yaw angles to a rotation matrix.
%
% Inputs:
%   roll  - Rotation about the x axis [rad].
%   pitch - Rotation about the y axis [rad].
%   yaw   - Rotation about the z axis [rad].
%
% Output:
%   rotationMatrix - [3x3] rotation matrix computed with the convention R = Rz(yaw)*Ry(pitch)*Rx(roll).

rotationMatrix = [cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);
                  sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
                  -sin(pitch),         cos(pitch)*sin(roll),                                    cos(pitch)*cos(roll)];
end
