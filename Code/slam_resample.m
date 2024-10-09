function [particles] = slam_resample(particles, init_weight)
	
	particles_count = size(particles, 2);
	weight_total = 0;
	
    % Calculating the total weight
    for i = 1:particles_count
		weight_total = weight_total + particles(i).weight;
    end

    % Create array to store newly resampled particles
    new_particles = particles;

	for i = 1:particles_count
        % Missing codes start here
        
        % Generate a number between the total weight and 0
        random_num = rand() * weight_total;

        % Resamples particles based on their weights
        cum_weight = 0;
        for k = 1:particles_count
            cum_weight = cum_weight + particles(k).weight;
            if cum_weight >= random_num
                % Select the following particle to be resamples
                new_particles(i) = particles(k);
                break;
            end
        end

        % Afterwards, each new partical should be given the same init_weight
        new_particles(i).weight = init_weight;

        % Missing codes end here
    end

    % Replace the old particles with new ones
    particles = new_particles;
 
end