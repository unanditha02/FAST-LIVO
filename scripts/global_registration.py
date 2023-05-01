#!/usr/bin/env python

from __future__ import print_function

from fast_livo.srv import global_registration
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Transform
import open3d as o3d
from global_registration import *
import numpy as np
import copy
from scipy.spatial.transform import Rotation as R

from ctypes import * # convert float to uint32

import rospy
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

import time
import rospy

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

def convertCloudFromRosToOpen3d(ros_cloud):
    
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4559,
                                      front=[0.6452, -0.3036, -0.7011],
                                      lookat=[1.9892, 2.0208, 1.8945],
                                      up=[-0.2779, -0.9482, 0.1556])

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 30
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 50
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(source, target, voxel_size):
    print(":: Load two point clouds and disturb initial pose.")
    # source = o3d.io.read_point_cloud("../../TestData/ICP/cloud_bin_0.pcd")
    # target = o3d.io.read_point_cloud("../../TestData/ICP/cloud_bin_1.pcd")
    # trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
    #                          [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    # source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result   #result.transformation is a 4x4 numpy array

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, result_fast):
    distance_threshold = voxel_size * 0.1
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result

def handle_global_registration(req):
    # print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    voxel_size = 0.05  # means 5cm for the dataset
    source = convertCloudFromRosToOpen3d(req.pcl1)
    target = convertCloudFromRosToOpen3d(req.pcl2)
    
    
    source, target, source_down, target_down, source_fpfh, target_fpfh = \
            prepare_dataset(source, target, voxel_size)
    
    result_fast = execute_fast_global_registration(source_down, target_down,
                                                   source_fpfh, target_fpfh,
                                                   voxel_size)
    
    result_icp = refine_registration(source_down, target_down, source_fpfh, target_fpfh,
                                     voxel_size, result_fast)
    # print("Fast global registration took %.3f sec.\n" % (time.time() - start))
    draw_registration_result(source_down, target_down,
                             result_icp.transformation)
    print(result_icp.transformation)
    rotation_mat = copy.deepcopy(result_icp.transformation[0:3, 0:3])
    r = R.from_matrix(rotation_mat)
    t = Transform()
    t.rotation.x = r.as_quat()[0]
    t.rotation.y = r.as_quat()[1]
    t.rotation.z = r.as_quat()[2]
    t.rotation.w = r.as_quat()[3]

    t.translation.x = result_icp.transformation[0,-1]
    t.translation.y = result_icp.transformation[1,-1]
    t.translation.z = result_icp.transformation[2,-1]

    return t

def global_registration_server():
    rospy.init_node('global_registration_server')
    s = rospy.Service('global_registration', global_registration, handle_global_registration)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    global_registration_server()