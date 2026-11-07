% A single multidimensional task at a given level
% This function computes the new rho value based on the Jacobian and
% reference value of this task, its activation function and the previous
% value of the projection matrix.
% It returns the new value of rho and the new value of the projection
% matrix
% Paramters:
% - A, the activation matrix, must be a diagonal matrix of values smoothly
% evolving from 0 to 1
% - J, the Jacobian matrix of this task
% - Qold, the projection matrix of the previous priority level
% - rhoold, the previous computed rho
% - xdot, the reference value of this task
% - lambda, threshold, the two parameters used in the singular value
% oriented regularization of A*J*Qold (see
% SingularValueOrientedRegularization.m)
%
% Return values:
% - Q, the new projection matrix
% - rho, the updated rho value
function [Q, rho, W, p1, p2, new_min, new_max] = iCAT_task(A, J, Qold, rhoold, xdot, lambda, threshold, weight, ...
    xmin, xmax, origin_xmin, origin_xmax, K)
   n = size(J, 2);
   
   if (size(A, 2) ~= size(J, 1))
       error('A and J have different size');
   end
   
   %JQpinv1 = iCAT_pseudoInverse(J*Qold, A, Qold, lambda, threshold, weight);
   %JQpinv2 = iCAT_pseudoInverse(J*Qold, A, eye(n),lambda, threshold, weight);
   [JQpinv1, p1] = iCAT_pseudoInverse(J*Qold, A, Qold, lambda, threshold, weight, K);
   [JQpinv2, p2] = iCAT_pseudoInverse(J*Qold, A, eye(n),lambda, threshold, weight, K);
  
   % compute the new projection matrix
   Q = Qold*(eye(n) - JQpinv2*J*Qold); 
   % compute W to smooth out the use of Qold in lower priority tasks
   W = J*Qold*JQpinv1;
   delta_y = Qold*JQpinv2*W*(xdot - J*rhoold);
   [delta_y_saturated, new_min, new_max] = SaturateVector(delta_y, xmin, xmax, origin_xmin, origin_xmax);
   
   % compute the new rho
   rho = rhoold + delta_y_saturated;
end