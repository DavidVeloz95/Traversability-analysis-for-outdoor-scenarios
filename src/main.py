import numpy as np
import open3d as o3d
import pye57 as p57

# Parameters
voxel_sz = 0.1
distance_threshold = 0.1
ground_distance_threshold = 0.30
max_angle_deg = 25.0
ransac_n = 3
num_iterations = 2500

# Robot dimensions
a, b, c = 0.3, 0.3, 0.3

def e57_file_to_open3d_object(e57):
    data = e57.read_scan_raw(0)
    xyz = np.column_stack((data["cartesianX"], data["cartesianY"], data["cartesianZ"]))
    valid = np.isfinite(xyz).all(axis=1)
    if "cartesianInvalidState" in data:
        valid &= np.asarray(data["cartesianInvalidState"]) == 0
    xyz = xyz[valid]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    if all(k in data for k in ("colorRed", "colorGreen", "colorBlue")):
        rgb = np.column_stack((data["colorRed"], data["colorGreen"], data["colorBlue"])).astype(np.float64)[valid]
        if rgb.size and rgb.max() > 1.0:
            rgb /= 255.0
        pcd.colors = o3d.utility.Vector3dVector(rgb)   
    return pcd

def segment_and_color_plane(downpcd, distance_threshold, ransac_n, num_iterations):
    plane_model, inliers = downpcd.segment_plane(distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations)
    inlier_cloud = downpcd.select_by_index(inliers)
    remaining_downpcd = downpcd.select_by_index(inliers, invert=True)
    return inlier_cloud, remaining_downpcd, plane_model, inliers

def draw_with_camera_json(geometries, json_filepath):
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Open3D - Vista Personalizada", width=1280, height=720)
    for geom in geometries:
        vis.add_geometry(geom)
    ctr = vis.get_view_control()
    param = o3d.io.read_pinhole_camera_parameters(json_filepath)
    ctr.convert_from_pinhole_camera_parameters(param)
    vis.run()
    vis.destroy_window()


# Change this directory to where the pcd is stored
e57 = p57.E57("C:/Users/DVeloz/Documents/Portfolio/Traversability-analysis-for-outdoor-scenarios/data/cloud_merged.e57")
pcd = e57_file_to_open3d_object(e57)
# Downsampling
downpcd = pcd.voxel_down_sample(voxel_size=voxel_sz)

# RANSAC for plane segmentation (Ground)
inlier_cloud_down, _, plane_model_1, _ = segment_and_color_plane(downpcd, distance_threshold, ransac_n, num_iterations)

# Plane equation: ax + by + cz + d = 0
a_1, b_1, c_1, d_1 = plane_model_1
normal_plane_1 = np.array([a_1, b_1, c_1])
normal_plane_1 /= np.linalg.norm(normal_plane_1)  # Normalizar vector

# Evaluation of the whole cloude
points = np.asarray(pcd.points)

# The absolute distance from EACH point to the ground plane
dist_to_plane = (points @ normal_plane_1 + d_1)

# Points related to ground
is_near_ground = np.abs(dist_to_plane) < ground_distance_threshold
ground_inliers = np.where(is_near_ground)[0]

# Normals estimation on the original cloud to check for tilts
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
normals = np.asarray(pcd.normals)

# Angle between the normal at the point and the normal to the ground
dot_products = np.clip(np.abs(np.dot(normals, normal_plane_1)), 0.0, 1.0)
angles_degrees = np.degrees(np.arccos(dot_products))

# Walkable ground = Points near the Y-axis with an acceptable slope (< 25°)
walkable_mask = is_near_ground & (angles_degrees < max_angle_deg)
walkable_indices = np.where(walkable_mask)[0]

# Walkable Cloud (GREEN)
walkable_cloud = pcd.select_by_index(walkable_indices)
walkable_cloud.paint_uniform_color([0, 1.0, 0])

# Obstacles
obstacle_mask = (dist_to_plane >= ground_distance_threshold) & (dist_to_plane <= (ground_distance_threshold + c))
obstacle_indices = np.where(obstacle_mask)[0]

# Obstacles Cloud (RED)
obstacle_cloud = pcd.select_by_index(obstacle_indices)
obstacle_cloud.paint_uniform_color([1.0, 0, 0])

# Rest of the scene (tall structures higher than the robot)
remaining_indices = np.where(~walkable_mask & ~obstacle_mask)[0]
high_structures_cloud = pcd.select_by_index(remaining_indices)

# Visualization
draw_with_camera_json([walkable_cloud, obstacle_cloud, high_structures_cloud], "C:/Users/DVeloz/Documents/Portfolio/Traversability-analysis-for-outdoor-scenarios/results/camera.json")