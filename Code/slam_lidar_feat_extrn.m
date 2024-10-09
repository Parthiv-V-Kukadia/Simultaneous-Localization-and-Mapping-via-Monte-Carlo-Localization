function [lines, corners] = slam_lidar_feat_extrn(points)
% Take LiDAR scan, returns line segments and corners
	
	readings_count = size(points, 2);
	% Extract lines by calling split-and-merge algorithm on segmented points 
	lines = [];
	start_index = 1;
	for i = 1:readings_count-1
		% Segmentation of points based on obviously not continuous parts 
		if (abs(points(i).r - points(i+1).r) > 200 || points(i).r < 20 || i == readings_count-1)
			if (start_index ~= i)
				lines = [lines slam_lidar_split_merge(points(start_index:i), 100)];
			end
			start_index = i+1;
		end
	end

	% Extract potential corners between consecutive line segments 
	corners = [];
	for i = 1:size(lines, 2)-1
		if (lines(i).p2.x == lines(i+1).p1.x && lines(i).p2.y == lines(i+1).p1.y)
            
			% Set minimum line length (both sides) of a valid corner
			length_min = 300;
			if (lines(i).length > length_min && lines(i+1).length > length_min)
                
                % Missing codes start here ...
                
                % Calculate the direction vector of the two lines
                v1 = [lines(i).p2.x - lines(i).p1.x, lines(i).p2.y - lines(i).p1.y];
                v2 = [lines(i+1).p2.x - lines(i+1).p1.x, lines(i+1).p2.y - lines(i+1).p1.y];

                % Calculate angle span of the corner
                angle = acos(dot(v1/norm(v1),v2/norm(v2))); % Normalize the vectors
                
                % Only use corner features having angle from 60 to 120 degrees 
				if (angle > deg2rad(60) && angle < deg2rad(120))
					% Calculate heading direction of the corner
                    heading1 = v1(1)/norm(v1) + v2(1)/norm(v2);
                    heading2 = v1(2)/norm(v1) + v2(2)/norm(v2);
                    heading = slam_in_pi(atan2(heading2, heading1) + pi);
                    
                % Missing codes end here ...
					
					% Calculate / design corner covariance matrix
					range_sigma = 0.02; % Proportion of measured range 
					angle_sigma = 0.005; % Radians
                    
                    corner_covariance = zeros(4, 4);
					corner_covariance(1,1) = (lines(i).p2.r^2)*(angle_sigma^2)*(sin(lines(i).p2.theta)^2) + ((range_sigma*lines(i).p2.r)^2)*(cos(lines(i).p2.theta)^2);
					corner_covariance(2,2) = (lines(i).p2.r^2)*(angle_sigma^2)*(cos(lines(i).p2.theta)^2) + ((range_sigma*lines(i).p2.r)^2)*(sin(lines(i).p2.theta)^2);
                    corner_covariance(3,3) = 0.01;
					corner_covariance(4,4) = 0.01;

					corners = [corners struct('x', lines(i).p2.x, 'y', lines(i).p2.y, 'heading', heading, 'angle', angle, 'covariance', corner_covariance)];
				end
			end
		end
	end
end