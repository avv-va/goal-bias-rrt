import matplotlib.pyplot as plt

def draw(graph, start, goal, x_range, y_range, obstacles, path):
    plt.xlim(x_range[0], x_range[1])
    plt.ylim(y_range[0], y_range[1])

    for edge in graph.edges:
        x = [edge.node_a.point[0], edge.node_b.point[0]]
        y = [edge.node_a.point[1], edge.node_b.point[1]]
        plt.plot(x, y, color="black", marker = 'o')

    for obstacle in obstacles:
        x, y = obstacle.exterior.xy
        plt.plot(x, y, color='red')

    plt.title(f"Path length: {len(path) - 1}")

    # draw path
    i = 0
    while i < len(path):
        point = path[i].point
        plt.plot([point[0]], [point[1]], marker = 'o', color = 'purple')
        if i == len(path) - 1:
            break
        next_point = path[i+1].point
        x = [point[0], next_point[0]]
        y = [point[1], next_point[1]]
        plt.plot(x, y, color='red')
        i += 1
    
    # draw start and goal
    plt.plot([start.point[0]], [start.point[1]], marker = 'o', color="green")
    plt.plot([goal.point[0]], [goal.point[1]], marker = 'o', color="blue")
    
    # plt.savefig("plots/plot.png")
    plt.show()