function saturatedVector = Saturate(vector, maxAbsComponent)
%SATURATE Bound a vector while preserving its direction.
%
% Inputs:
%   vector          - [n x 1] vector to be limited.
%   maxAbsComponent - Non-negative scalar upper bound on the absolute value of every component.
%
% Output:
%   saturatedVector - [n x 1] vector satisfying norm(saturatedVector, inf) <= maxAbsComponent.
%
% If any component exceeds the bound, all components are multiplied by the same factor. This preserves the direction of the input vector rather than clipping its components independently.

largestAbsComponent = max(abs(vector));
if largestAbsComponent > maxAbsComponent
    saturatedVector = vector*(maxAbsComponent/largestAbsComponent);
else
    saturatedVector = vector;
end
end
