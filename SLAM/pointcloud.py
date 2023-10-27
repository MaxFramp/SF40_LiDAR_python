from common import *
from visualize import render_cloud


def load_point_cloud(filename, voxel_size):
    pcd = o3d.io.read_point_cloud(filename)
    pcd_down_clean = clean_down_cloud(pcd,voxel_size)

    return pcd_down_clean


def load_point_clouds(filenames, voxel_size):
    pcds = []
    for filename in filenames:
        pcds.append(o3d.io.read_point_cloud(filename, voxel_size))

    return pcds


def filename_generator(n):
    filename, filenames = '', []
    for i in range(n):
        filename = 'data/cloud' + str(i) + '.xyz'
        filenames.append(filename)

    return filenames






def clean_down_cloud(cloud, voxel_size):
    cl, ind = cloud.remove_radius_outlier(nb_points=2, radius=20)
    cloud_clean = cloud.select_by_index(ind)
    cloud_down_clean = cloud_clean.voxel_down_sample(voxel_size)
    return cloud_down_clean


def segmentPointCloud(cloud, eps=25, min_points=2):
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            cloud.cluster_dbscan(eps, min_points, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    render_cloud(cloud)


