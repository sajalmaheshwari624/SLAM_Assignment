function updated_map = avgProjMapWithInputData(proj_map, input_data, alpha, h, w, is_use, t)

    %==== Set variables ====
    input_points = input_data.pointcloud.Location;
    input_colors = input_data.pointcloud.Color;
    input_normals = input_data.normals;
    proj_points = proj_map.points;
    proj_colors = proj_map.colors;
    proj_normals = proj_map.normals;
    proj_ccounts = proj_map.ccounts;
    proj_times = proj_map.times;

    %==== TODO: Update all the terms in the projected map using the input data ====
    %==== (Hint: apply is_use[] as a mask in vectorization) ====

    % Write your code here...
    indices = find(is_use == 1);
    updated_points = reshape(proj_points, [h*w,3]);
    updated_colors = reshape(proj_colors, [h*w,3]);
    updated_normals = reshape(proj_normals, [h*w,3]);
    updated_ccounts = reshape(proj_ccounts, [h*w,1]);
    updated_times = reshape(proj_times, [h*w,1]);
    updated_alpha = reshape(alpha, [h*w,1]);
    
    updated_input_points = reshape(input_points, [h*w,3]);
    updated_input_colors = reshape(input_colors, [h*w,3]);
    updated_input_normals = reshape(input_normals, [h*w,3]);
    
    updated_points(indices, :) = (updated_ccounts(indices).*updated_points(indices, :) + updated_alpha(indices).*updated_input_points(indices, :))./(updated_ccounts(indices) + updated_alpha(indices));
    updated_normals(indices, :) = (updated_ccounts(indices).*updated_normals(indices, :) + updated_alpha(indices).*updated_input_normals(indices, :))./(updated_ccounts(indices) + updated_alpha(indices));
    
    updated_colors(indices,:) = updated_input_colors(indices,:);
    updated_times(indices) = t;
    updated_ccounts(indices) = updated_ccounts(indices) + updated_alpha(indices);
    
    updated_points = reshape(updated_points, [h, w, 3]);
    updated_colors = reshape(updated_colors, [h, w, 3]);
    updated_normals = reshape(updated_normals, [h, w, 3]);
    updated_ccounts = reshape(updated_ccounts, [h, w, 1]);
    updated_times = reshape(updated_times, [h, w, 1]);
    
    %==== Output the updated projected map in a struct ====
    updated_map = struct('points', updated_points, 'colors', updated_colors, 'normals', updated_normals, 'ccounts', updated_ccounts, 'times', updated_times);
        
end