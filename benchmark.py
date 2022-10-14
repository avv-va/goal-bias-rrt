import matplotlib.pyplot as plt

from main import get_input_a, get_input_b, get_input_c
from rrt import GoalBiasRRT


def get_a_n_r_list():
    return [(200, 0.5), (200, 1), (200, 1.5), (200, 2), (500, 0.5), (500, 1), (500, 1.5), (500, 2)]


def get_b_c_n_r_list():
    return [(200, 1), (200, 2), (500, 1), (500, 2), (1000, 1), (1000, 2)]


def benchmark():
    p = 0.05  # goal bias probability
    epsilon = 0.25  # termination condition
    
    start, goal, x_range, y_range, obstacles = get_input_c()
    n_r_list = get_b_c_n_r_list()

    solutions = {}
    for n_r in n_r_list:
        solutions[n_r] = {
            False: 0,
            True: 0,
        }


    box_plot_path = []
    box_plot_comp = []
    where = []
    data = []
    labels = []


    for n_r in n_r_list:
        n, r = n_r[0], n_r[1]
        path_lengths = []
        computation_times = []

        for _ in range(0, 100):
            prm = GoalBiasRRT(n, r, p, epsilon, start, goal, x_range, y_range, obstacles)
            path, _, computation_time, _, _, _, found_solution = prm.path_planning()
            path_lengths.append(len(path) - 1)
            computation_times.append(computation_time)
            solutions[n_r][found_solution] += 1

        labels.append(n_r)
        box_plot_comp.append(computation_times)
        box_plot_path.append(path_lengths)
        
        where.append(f'{n_r}-s')
        where.append(f'{n_r}-f')
        data.append(solutions[n_r][True])
        data.append(solutions[n_r][False])


    plt.boxplot(box_plot_path, labels=labels)
    plt.title("path lengths")
    # plt.savefig(f"plots_again/path_lengths_benchmark.png")
    plt.show()
    plt.clf()
    plt.cla()
    plt.close()


    plt.boxplot(box_plot_comp, labels=labels)
    plt.title("computation times")
    # plt.savefig(f"plots_again/computation_times_benchmark.png")
    plt.show()
    plt.clf()
    plt.cla()
    plt.close()

    plt.bar(where, data)
    # plt.savefig("plots_again/solutions_benchmark.png")
    plt.show()


if __name__ == "__main__":
    benchmark()
