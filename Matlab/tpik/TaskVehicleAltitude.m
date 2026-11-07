classdef TaskVehicleAltitude < Task
    properties
        feedGain = 0.5;
        altMin = -38.0;
    end
    methods
        function updateReference(obj, robot)
            err_w = obj.altMin - robot.eta(3);
            
            obj.xdotbar = obj.feedGain * err_w;
        end
        
        function updateJacobian(obj, robot)
            wRv = robot.wTv(1:3, 1:3);
            
            obj.J = [zeros(3, 7), wRv, zeros(3, 3)];
        end
        
        function updateActivation(obj, robot)

            actAlt = robot.eta(3);

            if actAlt < obj.altMin
                a = 1;
            else 
                a = 0;
            end

            obj.A = a * eye(3);
        end
    end
end