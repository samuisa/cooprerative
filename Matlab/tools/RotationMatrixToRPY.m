function [roll, pitch, yaw] = RotationMatrixToRPY(rotationMatrix)
%ROTATIONMATRIXTORPY Convert a rotation matrix to roll, pitch and yaw angles.
%
% Input:
%   rotationMatrix - [3x3] rotation matrix using the convention R = Rz(yaw)*Ry(pitch)*Rx(roll).
%
% Outputs:
%   roll  - Rotation about the x axis [rad].
%   pitch - Rotation about the y axis [rad].
%   yaw   - Rotation about the z axis [rad].
%
% When the first column is aligned with the z axis, roll and yaw are not independently observable. The singular branch fixes yaw to zero and returns one valid representation.

singularityTolerance = 1e-12;
if abs(rotationMatrix(1,1)) < singularityTolerance && abs(rotationMatrix(2,1)) < singularityTolerance
    yaw = 0;
    pitch = -pi/2*rotationMatrix(3,1);
    roll = atan2(-rotationMatrix(1,2), rotationMatrix(2,2));
else
    yaw = atan2(rotationMatrix(2,1), rotationMatrix(1,1));
    sinYaw = sin(yaw);
    cosYaw = cos(yaw);
    pitch = atan2(-rotationMatrix(3,1), rotationMatrix(1,1)*cosYaw + rotationMatrix(2,1)*sinYaw);
    roll = atan2(rotationMatrix(1,3)*sinYaw - rotationMatrix(2,3)*cosYaw, -rotationMatrix(1,2)*sinYaw + rotationMatrix(2,2)*cosYaw);
end
end
