from shapely.geometry import Polygon
from rrt import GoalBiasRRT


def get_input_a():
    start = (0, 0)
    goal = (10, 0)
    x_range = (-1, 11)
    y_range = (-3, 3)
    obs1 = Polygon([(3.5, 1.5), (4.5, 1.5), (4.5, 0.5), (3.5, 0.5)])
    obs2 = Polygon([(6.5, -0.5), (7.5, -0.5), (7.5, -1.5), (6.5, -1.5)])
    return start, goal, x_range, y_range, [obs1, obs2]


def get_input_b():
    start = (0, 0)
    goal = (10, 10)
    x_range = (-1, 14)
    y_range = (-1, 14)
    o_1 = Polygon([(1, 1), (2, 1), (2, 5), (1, 5)])
    o_2 = Polygon([(3, 3), (4, 3), (4, 12), (3, 12)])
    o_3 = Polygon([(3, 12), (12, 12), (12, 13), (3, 13)])
    o_4 = Polygon([(12, 5), (13, 5), (13, 13), (12, 13)])
    o_5 = Polygon([(6, 5), (12, 5), (12, 6), (6, 6)])
    return start, goal, x_range, y_range, [o_1, o_2, o_3, o_4, o_5]


def get_input_c():
    start = (0, 0)
    goal = (35, 0)
    x_range = (-7, 36)
    y_range = (-7, 7)
    o_1 = Polygon([(-6, -6), (25, -6), (25, -5), (-6, -5)])
    o_2 = Polygon([(-6, 5), (30, 5), (30, 6), (-6, 6)])
    o_3 = Polygon([(-6, -5), (-5, -5), (-5, 5), (-6, 5)])
    o_4 = Polygon([(4, -5), (5, -5), (5, 1), (4, 1)])
    o_5 = Polygon([(9, 0), (10, 0), (10, 5), (9, 5)])
    o_6 = Polygon([(14, -5), (15, -5), (15, 1), (14, 1)])
    o_7 = Polygon([(19, 0), (20, 0), (20, 5), (19, 5)])
    o_8 = Polygon([(24, -5), (25, -5), (25, 1), (24, 1)])
    o_9 = Polygon([(29, 0), (30, 0), (30, 5), (29, 5)])
    return start, goal, x_range, y_range, [o_1, o_2, o_3, o_4, o_5, o_6, o_7, o_8, o_9]


if __name__ == "__main__":
    n = 5000  # maximum number of iterations
    r = 0.5  # step size
    
    p = 0.05  # goal bias probability
    epsilon = 0.25  # termination condition
    
    start, goal, x_range, y_range, obstacles = get_input_a()

    gbRRT = GoalBiasRRT(n, r, p, epsilon, start, goal,
                        x_range, y_range, obstacles)
    gbRRT.path_planning()
    gbRRT.draw_tree()
