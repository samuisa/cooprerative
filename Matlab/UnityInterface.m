classdef UnityInterface < handle
    %UNITYINTERFACE Exchange UVMS state and altitude with the Unity viewer.
    %
    % The interface sends arm joints and vehicle pose on two UDP channels and receives the latest altitude measurement on a third channel. Fixed rotations convert the simulator frame convention to the Unity convention. Network packets are transmitted as single-precision numeric arrays.
    
    properties (Access = private)
        uArm         % UDP output socket for the seven arm joints.
        uVehicle     % UDP output socket for the six vehicle pose values.
        uAltitude    % UDP input socket for altitude feedback.
        wuRw         % [3x3] simulator-world to Unity-world rotation.
        vRvu         % [3x3] Unity-vehicle to simulator-vehicle rotation.
        unityIP      % Destination IPv4 address or host name.
        armPort      % Destination port for arm data.
        vehiclePort  % Destination port for vehicle data.
        altitudePort % Local port receiving altitude data.
    end
    
    methods
        function obj = UnityInterface(unityIP)
            %UNITYINTERFACE Create and configure the three UDP sockets.
            %
            % Optional input:
            %   unityIP - String or character vector containing the Unity host address. Defaults to "127.0.0.1".
            %
            % Output:
            %   obj - Initialized UnityInterface handle object.
            if nargin < 1
                obj.unityIP = "127.0.0.1";
            else
                obj.unityIP = unityIP;
            end

            % Ports must match the corresponding Unity UDP components.
            obj.armPort     = 15000;
            obj.vehiclePort = 15001;
            obj.altitudePort= 15003;

            % Constant rotations reconcile axis order and sign conventions between the MATLAB simulation and the Unity scene.
            obj.wuRw = RPYToRotationMatrix(0, -pi/2, pi/2);
            obj.vRvu = RPYToRotationMatrix(-pi/2, 0, -pi/2);

            % The output datagram sizes are 7*4 bytes for the arm and 6*4 bytes for the vehicle because every transmitted value is single.
            obj.uArm = udpport("datagram", "IPV4", "ByteOrder", "big-endian", 'OutputDatagramSize', 28);
            obj.uVehicle = udpport("datagram", "IPV4", "ByteOrder", "big-endian", 'OutputDatagramSize', 24);
            obj.uAltitude = udpport("datagram", "IPV4", "LocalPort", obj.altitudePort);
        end

        function send(obj, uvms)
            %SEND Transmit the current arm configuration and vehicle pose.
            %
            % Input:
            %   uvms - UvmsModel containing q and wTv.
            %
            % Output:
            %   None. Two UDP datagrams are sent to the configured Unity host.

            % Send the seven arm joint coordinates without frame conversion.
            packetArm = single(uvms.q);
            write(obj.uArm, packetArm, "single", obj.unityIP, 15000);

            % Express vehicle attitude using Unity's world and body axes, then convert the rotation matrix to roll, pitch and yaw.
            unity_vehicle = obj.wuRw * uvms.wTv(1:3,1:3) * obj.vRvu;
            [r, p, y] = RotationMatrixToRPY(unity_vehicle);

            % Rotate the world-frame position into Unity axes. Additional sign changes below follow the Unity scene's handedness convention.
            trasl = uvms.wTv(1:3,4);
            unity_vehicle_trasl = obj.wuRw * trasl;

            packetVehicle = single([unity_vehicle_trasl(1), -unity_vehicle_trasl(2), unity_vehicle_trasl(3), -r, p, -y]);

            write(obj.uVehicle, packetVehicle, "single", obj.unityIP, 15001);
        end

        function altitude = receiveAltitude(obj, uvms)
            %RECEIVEALTITUDE Return the most recent Unity altitude measurement.
            %
            % Input:
            %   uvms - UvmsModel providing the previous altitude and vTw.
            %
            % Output:
            %   altitude - Scalar vertical distance resolved along the world vertical direction [m]. If no datagram is available, the previous uvms.altitude value is returned.

            % UDP feedback is asynchronous. Keeping the previous value avoids introducing a false zero measurement between incoming packets.
            altitude = uvms.altitude;

            if obj.uAltitude.NumDatagramsAvailable > 0
                % Drain the receive queue so control uses the freshest sample rather than accumulating latency from older datagrams.
                dgrams = read(obj.uAltitude, obj.uAltitude.NumDatagramsAvailable, "single");

                % Decode the payload of the latest datagram according to the numeric layout produced by the Unity sender.
                latestData = typecast(dgrams(end).Data, 'double');
                sensorDistance = latestData(end);

                % Project the Unity distance vector onto the simulator's world vertical. vTw supplies the current vehicle-to-world relation.
                w_kw = [0 0 1]';
                v_kw = uvms.vTw(1:3,1:3) * w_kw;
                altitude = v_kw' * [0 0 double(sensorDistance)]';
            end
        end

        function delete(obj)
            %DELETE Release UDP resources when the interface is destroyed.
            %
            % Input/output:
            %   None. The method clears the three private socket handles.
            %
            % Cleanup is deliberately guarded so that a partially constructed object can still be deleted without masking the original error.
            try
                clear obj.uArm obj.uVehicle obj.uAltitude
            catch
            end
        end
    end
end
