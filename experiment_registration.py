import numpy as np
import copy
import open3d as o3d

# pcd_data = o3d.data.DemoICPPointClouds()
source = o3d.io.read_point_cloud("./scanned_point_cloud_1.ply", format='ply')
target = o3d.io.read_point_cloud("./scanned_point_cloud_2.ply", format='ply')

voxel_size = 0.005
distance_threshold = 2 * voxel_size
source_down = source.voxel_down_sample(voxel_size)
target_down = target.voxel_down_sample(voxel_size)
print(f"{len(source.points)}->{len(source_down.points)},\n\
    {len(target_down.points)}->{len(target_down.points)}")

# estimate rough transformation using correspondences


#print("Compute a rough transform using the correspondences given by xx")
#p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
# trans_init = p2p.compute_transformation(source, target,
#                                        o3d.utility.Vector2iVector(corr))
# print(trans_init)

# point-to-point ICP for refinement
#print("Perform point-to-point ICP refinement")
# threshold = 0.03  # 3cm distance threshold
# reg_p2p = o3d.pipelines.registration.registration_icp(
#    source, target, threshold, trans_init,
#    o3d.pipelines.registration.TransformationEstimationPointToPoint())

print("Runing FPFH feature detection...")
feature_source = o3d.pipelines.registration.compute_fpfh_feature(
    source_down, o3d.geometry.KDTreeSearchParamHybrid(radius=distance_threshold * 5, max_nn=100))
target_source = o3d.pipelines.registration.compute_fpfh_feature(
    target_down, o3d.geometry.KDTreeSearchParamHybrid(radius=distance_threshold * 5, max_nn=100))
print(feature_source, target_source)


print('Running FGR ...')
result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
    source_down, target_down, feature_source, target_source,
    o3d.pipelines.registration.FastGlobalRegistrationOption(
        maximum_correspondence_distance=distance_threshold,  # m
        iteration_number=1000,
        maximum_tuple_count=1000))
print(result.transformation)


# print('Running RANSAC')
# result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
#     source_down,
#     target_down,
#     feature_source,
#     target_source,
#     mutual_filter=True,
#     max_correspondence_distance=distance_threshold,
#     estimation_method=o3d.pipelines.registration.
#     TransformationEstimationPointToPoint(False),  # no scaling
#     ransac_n=10,
#     checkers=[
#         o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
#             0.01),
#         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
#             0.01),
#         # o3d.pipelines.registration.CorrespondenceCheckerBasedOnNormal(0.5)

#     ],
#     criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
#         100000, 0.999))
# print(result.transformation)


trans_init = np.asarray([
    [0.60890023, - 0.78095758,  0.13908905, - 0.25561982],
    [-0.75289756, - 0.51376433,  0.41132892, - 0.51996377],
    [-0.24977145, - 0.35517808, - 0.90081228, 1.70299042],
    [0.,       0., - 0.,       1.]])

print("Running ICP")
result = o3d.pipelines.registration.registration_icp(
    source_down, target_down, distance_threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000))
# """
# TransformationEstimationPointToPoint
#  TransformationEstimationPointToPlane
#  TransformationEstimationForGeneralizedICP
#  TransformationEstimationForColoredICP
# """

# visualize the final result
source.paint_uniform_color([1, 0, 0])
target.paint_uniform_color([0, 1, 0])
o3d.visualization.draw([source.transform(result.transformation), target])
