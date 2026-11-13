classdef ActionManager < handle
    properties
        dt (1,1) double = 0.005; 
        transitionDuration (1,1) double = 2.0; % Transizione di 2 secondi
        currentActionName (1,1) string = ""; 
    end

    properties (Access = private)
        actions          
        dofArm           
        dofVehicle       
        dof              
        satMin           
        satMax           
        
        unifyingTaskList cell = {}
        previousActionName (1,1) string = ""
        transitionTimer double = inf
    end

    properties (Constant, Access = private)
        residualLambda = 1e-4
        residualThreshold = 0.01
        residualGain = 10
    end

    methods
        function obj = ActionManager(robot)
            obj.actions = configureDictionary("string", "cell");
            obj.dofArm = numel(robot.q);
            obj.dofVehicle = numel(robot.v_nu);
            obj.dof = obj.dofArm + obj.dofVehicle;

            armMax = robot.q_dot_max(:);
            vehicleMax = robot.v_nu_max(:);
            obj.satMax = [armMax; vehicleMax];
            obj.satMin = -obj.satMax;
        end

        function addAction(obj, actionName, taskStack)
            actionName = string(actionName);
            obj.actions(actionName) = {taskStack};
        end
        
        function setUnifyingTaskList(obj, taskList)
            obj.unifyingTaskList = taskList;
        end

        function setCurrentAction(obj, actionName)
            actionName = string(actionName);
            if ~isKey(obj.actions, actionName)
                error('ActionManager:UnknownAction', 'Action sconosciuta.');
            end
            
            if obj.currentActionName ~= "" && obj.currentActionName ~= actionName
                obj.previousActionName = obj.currentActionName;
            end
            obj.currentActionName = actionName;
            obj.transitionTimer = 0.0;
        end

        function [v_nu, q_dot] = computeICAT(obj, robot)
            if obj.currentActionName == ""
                error('ActionManager:NoActiveAction', 'Nessuna azione attiva.');
            end

            obj.transitionTimer = obj.transitionTimer + obj.dt;

            if isempty(obj.unifyingTaskList)
                tasks = obj.actions{obj.currentActionName};
            else
                tasks = obj.unifyingTaskList;
            end
            
            if ~isempty(obj.unifyingTaskList)
                taskNames = cellfun(@class, tasks, 'UniformOutput', false);
                currNames = cellfun(@class, obj.actions{obj.currentActionName}, 'UniformOutput', false);
                if obj.previousActionName == ""
                    prevNames = {};
                else
                    prevNames = cellfun(@class, obj.actions{obj.previousActionName}, 'UniformOutput', false);
                end
            end

            for i = 1:length(tasks)
                tasks{i}.updateReference(robot);
                tasks{i}.updateJacobian(robot);
                tasks{i}.updateActivation(robot);
            end

            ydotbar = zeros(obj.dof,1);
            Qp = eye(obj.dof);
            satMinRemaining = obj.satMin;
            satMaxRemaining = obj.satMax;
            K = eye(obj.dof); 
            
            for i = 1:length(tasks)
                t = tasks{i};
                
                if ~isempty(obj.unifyingTaskList)
                    inCurr = ismember(taskNames{i}, currNames);
                    inPrev = ismember(taskNames{i}, prevNames);
                    
                    if inCurr && inPrev
                        w = 1; 
                    elseif inCurr
                        w = IncreasingBellShapedFunction(0, obj.transitionDuration, 0, 1, obj.transitionTimer);
                    elseif inPrev
                        w = DecreasingBellShapedFunction(0, obj.transitionDuration, 0, 1, obj.transitionTimer);
                    else
                        w = 0; 
                    end
                    
                    A_eff = t.A * w;
                else
                    A_eff = t.A;
                end

                [Qp, ydotbar, ~, ~, ~, satMinRemaining, satMaxRemaining] = iCAT_task(A_eff, t.J, Qp, ydotbar, t.xdotbar, t.lambda, t.threshold, t.gain, satMinRemaining, satMaxRemaining, obj.satMin, obj.satMax, K);
            end

            [~, ydotbar] = iCAT_task(eye(obj.dof), eye(obj.dof), Qp, ydotbar, zeros(obj.dof,1), obj.residualLambda, obj.residualThreshold, obj.residualGain, satMinRemaining, satMaxRemaining, obj.satMin, obj.satMax, K);

            q_dot = ydotbar(1:obj.dofArm);
            v_nu = ydotbar(obj.dofArm+1:obj.dof);
        end
    end
end