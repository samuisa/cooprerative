classdef TaskVehicleAltitudeMin < Task
    properties
        feedGain = 0.5;
        altMin = 12.0;  % Limite minimo consentito
        buffer = 2.0;    % Zona di transizione prima di raggiungere il limite
    end
    
    methods
        function updateReference(obj, robot)
            if isnan(robot.altitude)
                obj.xdotbar = 0;
            else
                err_w = obj.altMin - robot.altitude; 
                obj.xdotbar = obj.feedGain * err_w;
            end
        end
        
        function updateJacobian(obj, robot)
            wRv = robot.wTv(1:3, 1:3); 
            obj.J = [zeros(1, 7), [0, 0, 1] * wRv, zeros(1, 3)];
        end
        
        function updateActivation(obj, robot)
            if isnan(robot.altitude)
                obj.A = 0; % Task inattivo finché non arriva il primo pacchetto
            else
                % Attivazione fluida basata sulla funzione a campana
                obj.A = DecreasingBellShapedFunction(...
                    obj.altMin, ...
                    obj.altMin + obj.buffer, ...
                    0, 1, robot.altitude);
            end
        end
    end
end