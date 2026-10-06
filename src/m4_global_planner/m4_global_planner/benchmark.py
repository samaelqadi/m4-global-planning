from statistics import mean, median, stdev

from m4_global_planner.astar import astar
from m4_global_planner.dummy_graph import create_dummy_graph
from m4_global_planner.metrics import run_with_metrics


def percentile(values, percent):
    # sort the values from smallest to largest
    values = sorted(values)

    # find the position of the requested percentile
    index = int((percent / 100) * (len(values) - 1))

    return values[index]


def run_benchmark(runs=100):
    planning_times = []
    cpu_times = []
    ram_values = []

    successful_runs = 0
    last_result = None

    for _ in range(runs):
        # create a fresh graph for every run
        graph, start_id, goal_id = create_dummy_graph()

        # run A* and measure it
        result = run_with_metrics(
            astar,
            graph,
            start_id,
            goal_id
        )

        last_result = result

        # count successful runs
        if result['success']:
            successful_runs += 1

        # save performance measurements
        planning_times.append(result['planning_time_ms'])
        cpu_times.append(result['cpu_time_ms'])
        ram_values.append(result['peak_ram_mb'])

    # calculate success percentage
    success_rate = (successful_runs / runs) * 100

    print('A* benchmark')
    print('Runs:', runs)
    print('Success rate:', round(success_rate, 2), '%')

    print()
    print('Planning time')
    print('Mean:', round(mean(planning_times), 4), 'ms')
    print('Median:', round(median(planning_times), 4), 'ms')
    print('P95:', round(percentile(planning_times, 95), 4), 'ms')
    print('Worst:', round(max(planning_times), 4), 'ms')

    if runs > 1:
        print('Standard deviation:', round(stdev(planning_times), 4), 'ms')

    print()
    print('CPU time')
    print('Mean:', round(mean(cpu_times), 4), 'ms')
    print('Worst:', round(max(cpu_times), 4), 'ms')

    print()
    print('Peak process RAM:', round(max(ram_values), 3), 'MB')

    if last_result is not None:
        print()
        print('Path:', last_result['path'])
        print('Total cost:', round(last_result['total_cost'], 3))
        print('Total distance:', round(last_result['total_distance'], 3), 'm')
        print('Walk distance:', round(last_result['walk_distance'], 3), 'm')
        print('Fly distance:', round(last_result['fly_distance'], 3), 'm')
        print('Transition distance:', round(last_result['transition_distance'], 3), 'm')
        print('Transitions:', last_result['transitions'])
        print('Nodes expanded:', last_result['nodes_expanded'])
        print('Nodes generated:', last_result['nodes_generated'])


if __name__ == '__main__':
    run_benchmark()
