from common import *


def render_cloud(clouds):
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    if type(clouds) == list:
        for cloud in clouds:
            vis.add_geometry(cloud)
    else:
        vis.add_geometry(clouds)

    vis.update_renderer()
    vis.run()
    vis.destroy_window()


def live_update_window():
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    for i in itertools.count(0):
        # do ICP single iteration
        # transform geometry using ICP
        # vis.update_geometry(geometry)
        vis.poll_events()
        vis.update_renderer()

