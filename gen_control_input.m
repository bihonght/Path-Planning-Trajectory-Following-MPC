% Generate a 2xN matrix
function [control] = gen_control_input(num_states)
    
    matrix = zeros(2, num_states);  % Generate a random 2xN matrix

    for j=1:num_states
        
    end

% Save the matrix to a file (e.g., 'matrix_data.mat')
    save('matrix_data.mat', 'matrix');