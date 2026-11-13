classdef TaskVehicleAltitudeControl < Task
    properties
        feedGain = 0.5;
        targetAltitude = 0.0; % Altitudine desiderata da mantenere
    end
    
    methods
        function updateReference(obj, robot)
            if isnan(robot.altitude)
                obj.xdotbar = 0; % Previene la propagazione di NaN
            else
                % Calcola l'errore rispetto all'altitudine attuale
                err_w = obj.targetAltitude - robot.altitude; 
                obj.xdotbar = obj.feedGain * err_w;
            end
        end
        
        function updateJacobian(obj, robot)
            wRv = robot.wTv(1:3, 1:3); 
            obj.J = [zeros(1, 7), [0, 0, 1] * wRv, zeros(1, 3)];
        end
        
        function updateActivation(obj, robot)
            if isnan(robot.altitude)
                obj.A = 0; % Mantiene il task inattivo senza dati del sensore
            else
                obj.A = 1; % Sempre attivo quando i dati sono validi
            end
        end
    end
end