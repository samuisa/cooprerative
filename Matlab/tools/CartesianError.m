function [angularError, linearError] = CartesianError(aTb, aTc)
%CARTESIANERROR Compute the pose error that brings frame c into frame b.
%
% Inputs:
%   aTb - [4x4] desired pose of frame b with respect to the common frame a.
%   aTc - [4x4] current pose of frame c with respect to the common frame a.
%
% Outputs:
%   angularError - [3x1] orientation error that rotates c toward b, expressed in frame a [rad].
%   linearError  - [3x1] position error that translates the origin of c toward the origin of b, expressed in frame a [m].
%
% Both transforms must be expressed relative to the same frame a. The outputs are geometric errors, not velocities; a feedback law converts them into angular and linear velocity references.

angularError = -VersorLemma(aTb(1:3,1:3), aTc(1:3,1:3));
linearError = aTb(1:3,4) - aTc(1:3,4);
end
