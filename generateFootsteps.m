function footsteps = generateFootsteps(p)
    footsteps = [0.0 p.step_width/2 0.0];
    for i = 1:p.num_footsteps
        footsteps = [footsteps; p.step_length_x*i/2 (-1)^i*p.step_width/2+p.step_length_y*i/2 0.0];
    end
end
