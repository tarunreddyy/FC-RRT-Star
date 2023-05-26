import trimesh
import numpy as np
import open3d as o3d
import pygame
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import math

from fc_rrt_star import FCRRTStar, Environment


def get_start_end_nodes(mesh):
    vertices = mesh.vertices
    top_down_coords = vertices[:, :2]
    min_coords = np.min(top_down_coords, axis=0)
    max_coords = np.max(top_down_coords, axis=0)
    normalized_coords = (top_down_coords - min_coords) / (max_coords - min_coords)

    pygame.init()
    width, height = 920, 920
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Select Start and Goal point")
    clock = pygame.time.Clock()

    scaling_factor = min(width / (max_coords[0] - min_coords[0]), height / (max_coords[1] - min_coords[1]))
    center_offset = np.array([(width - (max_coords[0] - min_coords[0]) * scaling_factor) / 2,
                              (height - (max_coords[1] - min_coords[1]) * scaling_factor) / 2])

    start_node_selected = False
    end_node_selected = False

    while True:
        screen.fill((255, 255, 255))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                if start_node_selected and end_node_selected:
                    return (start_node[0], 920 - start_node[1]), (end_node[0], 920 - end_node[1])
                else:
                    return None, None

            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_x, mouse_y = pygame.mouse.get_pos()
                selected_box = (mouse_x, mouse_y)

                if not start_node_selected:
                    start_node = selected_box
                    start_node_selected = True
                elif not end_node_selected:
                    end_node = selected_box
                    end_node_selected = True

        for coord in normalized_coords:
            x = int(coord[0] * width * scaling_factor) + int(center_offset[0])
            y = height - (int(coord[1] * height * scaling_factor) + int(center_offset[1]))
            pygame.draw.circle(screen, (0, 0, 0), (x, y), 5)

        if start_node_selected:
            pygame.draw.circle(screen, (0, 255, 0), (start_node[0], start_node[1]), 5)

        if end_node_selected:
            pygame.draw.circle(screen, (255, 0, 0), (end_node[0], end_node[1]), 5)

        # Update the display
        pygame.display.flip()
        clock.tick(60)

def explored_edges_to_lineset(tree, explored_nodes):
    points = []
    lines = []
    colors = []

    for i, node in enumerate(explored_nodes):
        parent = tree[node]
        if parent:
            points.append(node[:3])
            points.append(parent[:3])
            lines.append([2 * i, 2 * i + 1])
            colors.append([0, 1, 0])
    lineset = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    lineset.colors = o3d.utility.Vector3dVector(colors)

    return lineset

def nodes_to_pointcloud(nodes):
    pointcloud = o3d.geometry.PointCloud()
    pointcloud.points = o3d.utility.Vector3dVector([node[:3] for node in nodes])
    pointcloud.colors = o3d.utility.Vector3dVector([(1, 1, 1) for _ in nodes]) 
    return pointcloud

def custom_visualization(o3d_mesh, explored_edges_lineset, coordinate_axes, ground_plane, pointcloud, point_size=10):
    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window()

    visualizer.add_geometry(o3d_mesh)
    visualizer.add_geometry(explored_edges_lineset)
    visualizer.add_geometry(coordinate_axes)
    visualizer.add_geometry(ground_plane)
    visualizer.add_geometry(pointcloud)

    render_option = visualizer.get_render_option()
    render_option.point_size = point_size

    visualizer.run()
    visualizer.destroy_window()


mesh_data = trimesh.load_mesh("voxel_export.obj", process=False)

bounds = mesh_data.bounds
width = bounds[1][0] - bounds[0][0]
height = bounds[1][1] - bounds[0][1]
depth = bounds[1][2] - bounds[0][2]

start_node, end_node = get_start_end_nodes(mesh_data)

if start_node is not None and end_node is not None:
    print(f"Start node: {start_node}")
    print(f"End node: {end_node}")

    env = Environment(width, height, depth, mesh_data)
    start = (start_node[0], start_node[1], 100)
    goal = (end_node[0], end_node[1], 50)
    delta = 40 # node to node distance
    radius = 10 # number of nodes considered for rewiring

    max_turn_angle = math.radians(90)  
    max_climb_angle = math.radians(30)  
    fc_rrt_star = FCRRTStar(env, delta, radius, max_turn_angle, max_climb_angle)
    path = fc_rrt_star.generate_path(start, goal)

    if path is not None:
        path = [(int(round(point[0])), int(round(point[1])), int(round(point[2]))) for point in path]

        print("Path found:")
        for point in path:
            print(f"({point[0]}, {point[1]}, {point[2]})")

        explored_edges_lineset = explored_edges_to_lineset(tree=fc_rrt_star.tree, explored_nodes=fc_rrt_star.explored_nodes)

        o3d_mesh = o3d.geometry.TriangleMesh(
            vertices=o3d.utility.Vector3dVector(mesh_data.vertices),
            triangles=o3d.utility.Vector3iVector(mesh_data.faces)
        )

        cmap = plt.cm.get_cmap("hot")

        min_height = np.min(mesh_data.vertices[:, 2])
        max_height = np.max(mesh_data.vertices[:, 2])

        vertex_colors = np.zeros((len(mesh_data.vertices), 3))

        for i, vertex in enumerate(mesh_data.vertices):
            normalized_height = (vertex[2] - min_height) / (max_height - min_height)
            vertex_colors[i] = cmap(normalized_height)[:3]

        o3d_mesh.vertex_colors = o3d.utility.Vector3dVector(vertex_colors)

        points = nodes_to_pointcloud(path)
        coordinate_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=20)

        dimensions = bounds[1] - bounds[0]
        ground_plane = o3d.geometry.TriangleMesh.create_box(width=dimensions[0], height=dimensions[1], depth=0.01)
        ground_plane.translate([0, 0, 0])
        ground_plane.paint_uniform_color([0.5, 0.5, 0.5])

        custom_visualization(o3d_mesh, explored_edges_lineset, coordinate_axes, ground_plane, points, point_size=10)

    else:
        print("No path found.")
else:
    print("Start node and End node not selected.")