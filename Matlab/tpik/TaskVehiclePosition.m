classdef TaskVehiclePosition < Task
    properties
        feedGain = 0.5;
        tolerance = 1e-3; % Soglia di distanza [m] sotto cui il veicolo è considerato a target
    end
    methods
        function updateReference(obj, robot)
            err_w = robot.vehicleGoalPosition - robot.eta(1:3);
            
            obj.xdotbar = obj.feedGain * err_w;
        end
        
        function updateJacobian(obj, robot)
            wRv = robot.wTv(1:3, 1:3);
            
            obj.J = [zeros(3, 7), wRv, zeros(3, 3)];
        end
        
        function updateActivation(obj, robot)
            err_norm = norm(robot.vehicleGoalPosition - robot.eta(1:3));
            
            % Attivazione 0 se distanza <= 0.001 m, 1 se distanza >= 0.05 m
            a = IncreasingBellShapedFunction(1e-3, 0.05, 0, 1, err_norm);
            
            obj.A = a * eye(3);
        end
    end
end