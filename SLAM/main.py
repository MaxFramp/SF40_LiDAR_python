# This script needs to run on Python 3.10! The open3d python library does not exist on 3.11 as of 10/2023.
from common import *
from getLidarData import getDataTxt, getDataXYZ
from pointcloud import *
from ICP import *
from visualize import render_cloud


def run():
    k = 1  # Downsample the point cloud by only taking every k points
    num_samples = 50  # Number of samples (revolutions) of lidar data to include
    voxel_size = 15

    for i in range(1, num_samples + 2):
        getDataTxt(i+200)
        i += 1

    pcds_down = load_point_clouds(0, voxel_size)

    render_cloud(pcds_down)

    voxel_size = 25

    pcd_combined_down = transform_combine_clouds(0, 20, pcds_down, 40, num_samples)

    segmentPointCloud(pcd_combined_down, eps=30, min_points=2)


def loop():
    j = 0
    k = 1  # Downsample the point cloud by only taking every k points
    total_samples = 100  # Total number of revolutions to combine
    num_samples = 10  # Number of samples (revolutions) of lidar data to include per loop
    num_loops = int(total_samples / num_samples)
    init_voxel_size = 20
    loop_voxel_size = 20

    pcd_combined_clean_list = list()

    for i in range(1, total_samples+2):
        getDataXYZ(i)

    while j < num_loops:
        print(j)
        filenames = filename_generator(num_samples)
        pcds_down = load_point_clouds(filenames, init_voxel_size)

        # render_cloud(pcds_down)

        pcd_combined_down = transform_combine_clouds(j, 20, pcds_down, loop_voxel_size, num_samples)

        cl, ind = pcd_combined_down.remove_statistical_outlier(nb_neighbors=3, std_ratio=2.5)
        pcd_combined_clean = pcd_combined_down.select_by_index(ind)

        filename = 'data/cloud' + str(j) + '.xyz'
        o3d.io.write_point_cloud(filename, pcd_combined_clean)

        pcd_combined_clean_list.append(pcd_combined_clean)
        # segmentPointCloud(pcd_combined_clean, eps=40, min_points=2)

        j += 1

    render_cloud(pcd_combined_clean_list)
    pcd_all_down = transform_combine_all(0, 20, pcd_combined_clean_list, 40, num_loops)
    segmentPointCloud(pcd_all_down, eps=40, min_points=2)


def stream():  # Visualize the globally registered point cloud data, with updating master registry.
    master_cloud = o3d.geometry.PointCloud()
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(master_cloud)
    save_image = False
    combined_clean_list = list()

    for count in itertools.count(100,step=1):
        combined_clean_list.clear()
        map_list = list()
        j = 0
        k = 1  # Downsample the point cloud by only taking every k points
        total_samples = 50  # Total number of revolutions to combine
        num_samples_per_loop = 10  # Number of samples (revolutions) of lidar data to include per loop
        num_loops = int(total_samples / num_samples_per_loop)
        init_voxel_size = 30
        loop_voxel_size = 40

        for i in range(1, total_samples+2):
            getDataXYZ(count+i, num_rev=1, num_sets= num_samples_per_loop)

        while j < num_loops:
            filenames = filename_generator(num_samples_per_loop)
            pcds_down = load_point_clouds(filenames, init_voxel_size)

            # render_cloud(pcds_down)

            pcd_combined_down = transform_combine_clouds(j, 20, pcds_down, loop_voxel_size, num_samples_per_loop)

            cl, ind = pcd_combined_down.remove_statistical_outlier(nb_neighbors=3, std_ratio=2.5)
            pcd_combined_clean = pcd_combined_down.select_by_index(ind)
            pcd_combined_clean.voxel_down_sample(80)

            # filename = 'data/cloud' + str(j) + '.xyz'
            # o3d.io.write_point_cloud(filename, pcd_combined_clean)

            combined_clean_list.append(pcd_combined_clean)


            j += 1

        combined_clean_list.append(master_cloud)
        vis.remove_geometry(master_cloud)
        master_cloud = transform_combine_all(0, 20, combined_clean_list, 40, num_loops)
        master_cloud.voxel_down_sample(80)
        vis.add_geometry(master_cloud)

        vis.update_geometry(master_cloud)
        vis.poll_events()
        vis.update_renderer()
        vis.capture_screen_image("./figures/temp_%04d.jpg" % count)

    vis.destroy_window()
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)



if __name__ == '__main__':
    start_time = time.time()
    # run()
    loop()  # Loop achieve almost identical results in 1/3 of the time.
    # stream()

    print("time for [loop()] --- %s seconds ---" % (time.time() - start_time))

