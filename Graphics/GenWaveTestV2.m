% Parameters for the wave
num_frames = 32;
num_x = 24;
num_y = 24;
num_z = 32;

% Initialize arrays to store the data
frames = 1:num_frames;
x_array = 0:num_x-1;
y_array = 0:num_y-1;
z_array = 0:num_z-1;
LED_states = zeros(num_frames,num_x,num_y,num_z);
% Generate the wave
for frame = 1:num_frames
    frame
    for x = 0:num_x-1  % Adjusted for 0-based indexing
        for y = 0:num_y-1  % Adjusted for 0-based indexing
            
            % Calculate the z position of the wave
            z = round((num_z - 1) / 2 * (1 + sin(2 * pi * (x / num_x + y / num_y) + 2 * pi * frame / num_frames)));
            
            % Ensure z is within bounds [0, num_z-1]
            z = max(0, min(z, num_z - 1));
            
            % Determine the LED state (color) based on the z position
            if z < 8
                led_state = 1;  % Mix of red and green (yellow)
            elseif z < 16
                led_state = 2;  % Green
            elseif z < 24
                led_state = 3;  % Blue
            else
                led_state = 4;  % White
            end
            
            % save data for current frame
            LED_states(frame,x+1,y+1,z+1) = led_state;
        end
    end
end
values = LED_states(:);
[f_idx, x_idx, y_idx, z_idx] = ndgrid(1:num_frames, 1:num_x, 1:num_y, 1:num_z);
% Flatten the indices arrays
f_idx = f_idx(:);
x_idx = x_idx(:);
y_idx = y_idx(:);
z_idx = z_idx(:);

% Combine indices and values
data_matrix = [f_idx, x_idx-1, y_idx-1, z_idx-1, values];

T = array2table(data_matrix, 'VariableNames', {'Frame', 'X', 'Y', 'Z', 'LED'});
% Write the table to an Excel file
filename = 'wave_animation.csv';
writetable(T, filename);

disp(['Excel file created: ', filename]);
