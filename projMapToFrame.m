function [proj_map, proj_flag] = projMapToFrame(fusion_map, h, w, tform, cam_param)
    
    %==== Set parameters ====
    fx = cam_param(1);
    fy = cam_param(2);
    cx = cam_param(3);
    cy = cam_param(4);

    %==== TODO: Project all terms in the fusion map based on other input parameters ====
    %==== (Hint 1: Only project the points in front of the camera) ====
    %==== (Hint 2: Calculate all the projected indices together and use them in vectorization) ====
    %==== (Hint 3: Discard the indices that exceed the frame boundaries) ====

    % Write your code here...
    K = [fx 0 cx; 0 fy cy; 0 0 1];
    mapCameraFrame = pctransform(fusion_map.pointcloud, invert(tform));
    mapCameraZVal = mapCameraFrame.Location(:,3);
    validIndices1 = find(mapCameraZVal > 0);
    validCameraFrameMap = zeros(size(mapCameraFrame.Location));
    validCameraFrameMap(validIndices1,:) = (mapCameraFrame.Location(validIndices1,:));
    pointsProjectedHomo = K*validCameraFrameMap';
    pointsProjected2D = pointsProjectedHomo./ pointsProjectedHomo(3,:);
    pointsProjected2DValid = round(pointsProjected2D(1:2, :));
    validIndices2 = find(pointsProjected2DValid(1,:) > 0 & pointsProjected2DValid(1,:) < h);
    validIndices3 = find(pointsProjected2DValid(2,:) > 0 & pointsProjected2DValid(2,:) < w);
    validIndices = intersect(validIndices2, validIndices3);
    numValidPoints = numel(validIndices);
    proj_flag = false(numel(mapCameraZVal),1);
    proj_flag(validIndices) = 1;
    
    proj_points = zeros(h*w,3);
    proj_colors = zeros(h*w,3);
    proj_normals = zeros(h*w,3);
    proj_ccounts = zeros(h*w,1);
    proj_times = zeros(h*w,1);

    proj_points(pointsProjected2DValid(:,validIndices),:) = fusion_map.pointcloud.Location(pointsProjected2DValid(:,validIndices),:);
    proj_colors(pointsProjected2DValid(:,validIndices),:) = fusion_map.pointcloud.Color(pointsProjected2DValid(:,validIndices),:);
    proj_normals(pointsProjected2DValid(:,validIndices),:) = fusion_map.normals(pointsProjected2DValid(:,validIndices),:);
    proj_ccounts(pointsProjected2DValid(:,validIndices),:) = fusion_map.ccounts(pointsProjected2DValid(:,validIndices),:);
    proj_times(pointsProjected2DValid(:,validIndices),:) = fusion_map.times(pointsProjected2DValid(:,validIndices),:);
    
    proj_points = reshape(proj_points,[h,w,3]);
    proj_colors = reshape(proj_colors,[h,w,3]);
    proj_normals = reshape(proj_normals,[h,w,3]);
    proj_ccounts = reshape(proj_ccounts,[h,w]);
    proj_times = reshape(proj_times,[h,w]);
    
    %==== Output the projected map in a struct ====
    %==== (Notice: proj_points[], proj_colors[], and proj_normals[] are all 3D matrices with size h*w*3) ====
    %==== (Notice: proj_ccounts[] and proj_times[] are both 2D matrices with size h*w) ====
    proj_map = struct('points', proj_points, 'colors', proj_colors, 'normals', proj_normals, 'ccounts', proj_ccounts, 'times', proj_times);
        
end
