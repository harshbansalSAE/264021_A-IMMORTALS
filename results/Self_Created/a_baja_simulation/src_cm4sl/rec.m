% MATLAB Script to receive Radar Data
clear;
u = udpport("LocalHost", "127.0.0.1", "LocalPort", 5005);

fprintf('Waiting for Radar Data from Python...\n');

while true
    if u.NumBytesAvailable > 0
        % Read the packet as 3 doubles (8 bytes each)
        data = read(u, 3, "double");
        
        nDetections = data(1);
        ClosestDist = data(2);
        ClosestVel  = data(3);
        
        % Display values
        fprintf('Dets: %d | Dist: %.2f m | Vel: %.2f m/s\n', ...
                nDetections, ClosestDist, ClosestVel);
    end
    pause(0.01); % Prevent CPU hogging
end