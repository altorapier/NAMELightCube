% Parameters for the wave
num_frames = 32;
num_x = 24;
num_y = 24;
num_z = 32;

% Initialize arrays to store the data
Frame = [];
X = [];
Y = [];
Z = [];
LED = [];

% Generate the wave
for frame = 1:num_frames
    for x = 0:num_x-1  % Adjusted for 0-based indexing
        for y = 0:num_y-1  % Adjusted for 0-based indexing
            % Calculate the z position of the wave
            z = round((num_z - 1) / 2 * (1 + sin(2 * pi * (x / num_x + y / num_y) + 2 * pi * frame / num_frames)));
            
            % Ensure z is within bounds [0, num_z-1]
            z = max(0, min(z, num_z - 1));
            
            % Determine the LED state (color) based on the z position
            if z < 10
                led_state = 1;  % Mix of red and green
            elseif z < 21
                led_state = 2;  % Green
            else
                led_state = 3;  % Mix of green and blue
            end
            
            % Append the data for the current point
            Frame = [Frame; frame];
            X = [X; x];
            Y = [Y; y];
            Z = [Z; z];
            LED = [LED; led_state];
        end
    end
end

% Create a table to hold the data
T = table(Frame, X, Y, Z, LED);

% Write the table to an Excel file
filename = 'wave_animation.xlsx';
writetable(T, filename);

disp(['Excel file created: ', filename]);
