classdef TaskVehicleOrientation< Task
    properties
        feedGain = 1.5;
    end
    methods
        function updateReference(obj, robot)
            
            % z_v = robot.wTv(1:3, 3);
            
            % Usa le parentesi tonde e solo il simbolo + (senza virgole)
            err = (1/2) * (cross(robot.wTv(1:3, 1), [1; 0; 0]) + ...
                           cross(robot.wTv(1:3, 2), [0; 1; 0]) + ...
                           cross(robot.wTv(1:3, 3), [0; 0; 1]));
            
            obj.xdotbar = obj.feedGain * err;
        end
        
        function updateJacobian(obj, robot)
            wRv = robot.wTv(1:3, 1:3);
            
            % Jacobian 3x13 (7 arm joints + 3 vehicle translations + 3 vehicle rotation)
            obj.J = [zeros(3, 7), zeros(3, 3), wRv];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end