classdef TaskVehicleMisalignment< Task
    properties
        feedGain = 1.5;
        max_err = 0.20;
    end
    methods
        function updateReference(obj, robot)
            
            z_v = robot.wTv(1:3, 3);
            
            z_w = [0; 0; 1];
            
            err = cross(z_v, z_w);
            
            obj.xdotbar = obj.feedGain * err;
        end
        
        function updateJacobian(obj, robot)
            wRv = robot.wTv(1:3, 1:3);
            
            % Jacobian 3x13 (7 arm joints + 3 vehicle translations + 3 vehicle rotation)
            obj.J = [zeros(3, 7), zeros(3, 3), wRv];
        end
        
        function updateActivation(obj, robot)

            z_v = robot.wTv(1:3, 3);
            z_w = [0; 0; 1];

            misalignment_norm = norm(cross(z_v, z_w));
            
            % activation logic:
            % if the norm of misalignment error is > 0.2 task is ACTIVATED
            % otherwise it gets DEACTIVATED
            if misalignment_norm >= obj.max_err
                a = 1; % activation
            else
                a = 0; % deactivation
            end
            obj.A = a * eye(3);
        end
    end
end