import numpy as np
import math
import trimesh
import trimesh.voxel.creation
import trimesh.proximity

class Environment:
    def __init__(self, width, height, depth, mesh):
        self.width = width
        self.height = height
        self.depth = depth
        self.mesh = mesh
        self.proximity = trimesh.proximity.ProximityQuery(mesh)

    def is_collision(self, point):
        return self.mesh.contains(np.array([point[:3]]))

    def is_collision_free(self, start, end):
        intersections = self.mesh.ray.intersects_location(
            ray_origins=[start], ray_directions=[np.array(end) - np.array(start)], multiple_hits=True
        )
        return len(intersections[0]) == 0

def threat_strength(point, env, distance_threshold=30):
    point_3D = np.array([point[:3]])
    distance = env.proximity.signed_distance(point_3D)[0]
    return max(0, (distance - distance_threshold))

def flight_cost(path, env, w1=1, w2=1):
    cost = 0
    for i in range(len(path) - 1):
        segment_length = math.sqrt((path[i][0] - path[i+1][0]) ** 2 + (path[i][1] - path[i+1][1]) ** 2 + (path[i][2] - path[i+1][2]) ** 2)
        segment_threat = threat_strength(path[i], env)
        cost += w1 * segment_threat + w2 * segment_length
    return cost

class FCRRTStar:
    def __init__(self, env, delta, radius, max_turn_angle, max_climb_angle):
        self.env = env
        self.delta = delta
        self.radius = radius
        self.max_turn_angle = max_turn_angle
        self.max_climb_angle = max_climb_angle
        self.tree = {}

    def generate_path(self, start, goal, max_iterations=20000, goal_distance_threshold=10.0):
        self.tree = {start: None} 
        path = []
        iterations = 0
        self.explored_nodes = []
        nearest_goal_distance = float("inf")

        while not path and iterations < max_iterations:
            sample = self.sample(goal)
            nearest_node = self.nearest(self.tree, sample)
            new_node = self.steer(nearest_node, sample)
            if new_node and not self.env.is_collision(new_node) and self.env.is_collision_free(nearest_node[:3], new_node[:3]):
                self.tree[new_node] = nearest_node
                self.explored_nodes.append(new_node)
                self.rewire(self.tree, new_node)
                current_goal_distance = self.distance(new_node, goal)
                if current_goal_distance < nearest_goal_distance:
                    nearest_goal_distance = current_goal_distance
                    nearest_goal_node = new_node

                if self.reached_goal(new_node, goal):
                    path = self.reconstruct_path(self.tree, start, new_node)
            iterations += 1

        if not path and nearest_goal_distance <= goal_distance_threshold:
            path = self.reconstruct_path(self.tree, start, nearest_goal_node)

        return path if path else None

    def sample(self, goal, bias=0.1):
        if np.random.rand() < bias:
            offset = np.random.normal(scale=self.delta, size=3)
            x, y, z = np.array(goal[:3]) + offset
        else:
            x = np.random.uniform(0, self.env.width)
            y = np.random.uniform(0, self.env.height)
            z = np.random.uniform(0, self.env.depth)
        threat = threat_strength((x, y, z), self.env)
        return (x, y, z, threat)

    def nearest(self, tree, sample):
        return min(tree.keys(), key=lambda node: self.distance(node, sample))

    def steer(self, nearest_node, sample):
        if self.distance(nearest_node, sample) <= self.delta:
            return sample
        else:
            vector = np.array(sample[:3]) - np.array(nearest_node[:3])
            vector = vector / np.linalg.norm(vector)
            new_point = nearest_node[:3] + self.delta * vector

            angle = math.acos(np.dot(vector, np.array(nearest_node[:3])) / (np.linalg.norm(vector) * np.linalg.norm(nearest_node[:3])))
            if angle > self.max_turn_angle:
                return None

            climb_angle = math.atan((new_point[2] - nearest_node[2]) / self.distance(nearest_node, new_point))
            if abs(climb_angle) > self.max_climb_angle:
                return None

            new_x, new_y, new_z = new_point
            new_threat = 0 
            return (new_x, new_y, new_z, new_threat)
        
    def rewire(self, tree, new_node):
        neighbors = self.get_neighbors(tree, new_node)

        for neighbor in neighbors:
            if not self.is_descendant(tree, new_node, neighbor) and \
                self.cost(tree, new_node) + self.distance(new_node, neighbor) < self.cost(tree, neighbor) and \
                self.env.is_collision_free(new_node[:3], neighbor[:3]):
                tree[neighbor] = new_node

    def is_descendant(self, tree, current, node):
        while current:
            if current == node:
                return True
            current = tree[current]
        return False

    def get_neighbors(self, tree, node):
        return [n for n in tree if self.distance(n, node) <= self.radius]

    def reached_goal(self, node, goal):
        return self.distance(node, goal) <= self.delta

    def reconstruct_path(self, tree, start, goal):
        path = [goal]
        current = goal
        while current != start:
            current = tree[current]
            path.append(current)
        return path[::-1]

    def cost(self, tree, node):
        path = []
        current = node
        visited_nodes = set() 

        while current:
            if current in visited_nodes: 
                print(f"Loop detected in the tree: {current}")
                break
            visited_nodes.add(current) 

            path.append(current)
            current = tree[current]

        return flight_cost(path, self.env)

    def distance(self, node1, node2):
        return math.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2 + (node1[2] - node2[2])**2)
