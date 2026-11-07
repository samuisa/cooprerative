classdef TaskVehiclePosition < Task
    properties
        feedGain = 0.5;
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
            obj.A = eye(3);
        end
    end
end