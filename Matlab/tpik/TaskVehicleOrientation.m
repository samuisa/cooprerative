classdef TaskVehicleOrientation < Task
    properties
        feedGain = 1.5;
        tolerance = 0.05; 
    end
    
    methods
        function updateReference(obj, robot)
            goal_orientation = robot.wRgv;
            
            err = (1/2) * (cross(robot.wTv(1:3, 1), goal_orientation(1:3, 1)) + ...
                           cross(robot.wTv(1:3, 2), goal_orientation(1:3, 2)) + ...
                           cross(robot.wTv(1:3, 3), goal_orientation(1:3, 3)));
            
            obj.xdotbar = obj.feedGain * err;
        end
        
        function updateJacobian(obj, robot)
            wRv = robot.wTv(1:3, 1:3);
            
            obj.J = [zeros(3, 7), zeros(3, 3), wRv];
        end
        
        function updateActivation(obj, robot)
            goal_orientation = robot.wRgv;
            
            err = (1/2) * (cross(robot.wTv(1:3, 1), goal_orientation(1:3, 1)) + ...
                           cross(robot.wTv(1:3, 2), goal_orientation(1:3, 2)) + ...
                           cross(robot.wTv(1:3, 3), goal_orientation(1:3, 3)));
            
            if norm(err) > obj.tolerance
                a = 1;
            else 
                a = 0;
            end
            
            obj.A = a * eye(3);
        end
    end
end