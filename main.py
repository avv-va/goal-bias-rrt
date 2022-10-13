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


if __name__ == "__main__":
    n = 5000  # maximum number of iterations
    r = 0.5  # step size
    p = 0.05  # goal bias probability
    epsilon = 0.25  # termination condition
    start, goal, x_range, y_range, obstacles = get_input_a()
    gbRRT = GoalBiasRRT(n, r, p, epsilon, start, goal, x_range, y_range, obstacles)
    gbRRT.path_planning()
    gbRRT.draw_tree()

