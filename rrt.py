from shapely.geometry import LineString

import random
import math

from models import Node, Edge, Tree
from draw import draw
from a_star import a_star


class GoalBiasRRT:
    def __init__(self, n, r, p, epsilon, start, goal, x_range, y_range, obstacles) -> None:
        self.n = n
        self.r = r
        self.p = p
        self.epsilon = epsilon
        self.start = start
        self.goal = goal
        self.x_range = x_range
        self.y_range = y_range
        self.obstacles = obstacles

    def sample(self):
        sampled = random.choices(
            ['goal', 'obstacle'], weights=(self.p, 1-self.p))
        if sampled == 'goal':
            return self.goal_node
        x_sampled = round(random.uniform(self.x_range[0], self.x_range[1]), 3)
        y_sampled = round(random.uniform(self.y_range[0], self.y_range[1]), 3)
        return Node((x_sampled, y_sampled))

    def find_nearest(self, nodes, sampled_node):
        shortest_dist = math.inf
        nearest_node = None
        for node in nodes:
            distance = math.dist(node.point, sampled_node.point)
            if distance < shortest_dist:
                shortest_dist = distance
                nearest_node = node
        return nearest_node

    def generate_sub_path(self, nearest_node, sampled_node):
        possible_path = LineString([(nearest_node.point[0], nearest_node.point[1]), (sampled_node.point[0], sampled_node.point[1])])
        startpoint = nearest_node.point
        point_on_path = possible_path.interpolate(self.r)
        new_line = LineString([startpoint, point_on_path])
        return new_line

    def is_colision_free(self, sub_path):
        for obs in self.obstacles:
            if obs.intersects(sub_path):
                return False
        return True

    def path_planning(self):
        self.start_node = Node(self.start)
        self.goal_node = Node(self.goal)
        nodes = [self.start_node]
        edges = []

        iteration = 0
        while iteration <= self.n:
            sampled_node = self.sample()
            nearest_node = self.find_nearest(nodes, sampled_node)
            sub_path = self.generate_sub_path(nearest_node, sampled_node)
            if self.is_colision_free(sub_path):
                end_point_sub_path = list(zip(*sub_path.coords.xy))[1]
                new_node = Node(end_point_sub_path)
                
                new_edge1 = Edge(nearest_node, new_node)
                new_edge2 = Edge(new_node, nearest_node) # bidirection

                nodes.append(new_node)

                edges.append(new_edge1)
                edges.append(new_edge2)
            
                if math.dist(new_node.point, self.goal_node.point) <= self.epsilon:
                    new_edge = Edge(new_node, self.goal_node)
                    edges.append(new_edge)
                    break
            iteration += 1
        
        self.tree = Tree(nodes, edges)
        self.path, self.iterations, self.computation_time, self.found_solution = a_star(self.tree, self.start_node, self.goal_node)
        return self.path, self.iterations, self.computation_time, self.tree, self.start_node, self.goal_node, self.found_solution

    def draw_tree(self):
        draw(self.tree, self.start_node, self.goal_node, self.x_range, self.y_range, self.obstacles, self.path)

