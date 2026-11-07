function [saturatedVector, updatedMin, updatedMax] = SaturateVector(vector, currentMin, currentMax, originalMin, originalMax)
%SATURATEVECTOR Scale an iCAT velocity increment and update the residual limits.
%
% Inputs:
%   vector      - [n x 1] velocity increment proposed by the current task.
%   currentMin  - [n x 1] lower velocity limits still available before applying this increment.
%   currentMax  - [n x 1] upper velocity limits still available before applying this increment.
%   originalMin - [n x 1] original lower system-velocity limits.
%   originalMax - [n x 1] original upper system-velocity limits.
%
% Outputs:
%   saturatedVector - [n x 1] feasible increment obtained with a single scale factor, so its direction is preserved.
%   updatedMin      - [n x 1] lower limits left available to lower-priority tasks.
%   updatedMax      - [n x 1] upper limits left available to lower-priority tasks.
%
% The smallest component-wise feasible scale factor is applied to the complete vector. After subtracting the accepted increment, the current algorithm retains at least 10 percent of each original bound for the following priority levels.

scaleFactor = 1;
for i = 1:numel(vector)
    componentScaleFactor = 1;
    if vector(i) > currentMax(i) && vector(i) ~= 0
        componentScaleFactor = abs(currentMax(i)/vector(i));
    elseif vector(i) < currentMin(i) && vector(i) ~= 0
        componentScaleFactor = abs(currentMin(i)/vector(i));
    end
    scaleFactor = min(scaleFactor, componentScaleFactor);
end

saturatedVector = vector*scaleFactor;
updatedMin = zeros(size(vector));
updatedMax = zeros(size(vector));
for i = 1:numel(vector)
    remainingMax = currentMax(i) - saturatedVector(i);
    remainingMin = currentMin(i) - saturatedVector(i);
    updatedMin(i) = min(remainingMin, 0.1*originalMin(i));
    updatedMax(i) = max(remainingMax, 0.1*originalMax(i));
end
end
