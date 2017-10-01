import matplotlib.pyplot as plt
import pandas as pd
import os as os
import numpy as np

colors = plt.cm.Set1(np.linspace(0, 1, 9))
colors_pastel = plt.cm.Pastel1(np.linspace(0, 1, 9))

def process_matching_time(scan_matcher_df):
    if x_axis == 'translation':
        initial_errors = np.sqrt(np.square(scan_matcher_df.initial_error_x.as_matrix())
                                 + np.square(scan_matcher_df.initial_error_y.as_matrix())
                                 + np.square(scan_matcher_df.initial_error_z.as_matrix()))
    else:
        initial_errors = scan_matcher_df.initial_error_angle.as_matrix()
    initial_errors = np.reshape(initial_errors, (-1, batch_size))
    times = scan_matcher_df.time_scan_matching.as_matrix()
    times = np.reshape(times, (-1, batch_size))
    mean_initial_errors = np.mean(initial_errors, 1)
    mean_times = np.mean(times, 1)
    var_times = np.std(times, 1)
    return mean_initial_errors, mean_times, var_times


def process_map_update_time(scan_matcher_df):
    if x_axis == 'translation':
        initial_errors = np.sqrt(np.square(scan_matcher_df.initial_error_x.as_matrix())
                                 + np.square(scan_matcher_df.initial_error_y.as_matrix())
                                 + np.square(scan_matcher_df.initial_error_z.as_matrix()))
    else:
        initial_errors = scan_matcher_df.initial_error_angle.as_matrix()
    initial_errors = np.reshape(initial_errors, (-1, batch_size))
    times = scan_matcher_df.time_map_update.as_matrix()
    times = np.reshape(times, (-1, batch_size))
    mean_initial_errors = np.mean(initial_errors, 1)
    mean_times = np.mean(times, 1)
    var_times = np.std(times, 1)
    return mean_initial_errors, mean_times, var_times


def process_matching_error(scan_matcher_df):
    if x_axis == 'translation':
        initial_errors = np.sqrt(np.square(scan_matcher_df.initial_error_x.as_matrix())
                                 + np.square(scan_matcher_df.initial_error_y.as_matrix())
                                 + np.square(scan_matcher_df.initial_error_z.as_matrix()))
    else:
        initial_errors = scan_matcher_df.initial_error_angle.as_matrix()
    initial_errors = np.reshape(initial_errors, (-1, batch_size))
    matched_errors = np.sqrt(np.square(scan_matcher_df.matched_error_x.as_matrix())
                             + np.square(scan_matcher_df.matched_error_y.as_matrix())
                             + np.square(scan_matcher_df.matched_error_z.as_matrix()))
    matched_errors = np.reshape(matched_errors, (-1, batch_size))
    mean_initial_errors = np.mean(initial_errors, 1)
    mean_matched_errors = np.mean(matched_errors, 1)
    var_matched_errors = np.std(matched_errors, 1)
    return mean_initial_errors, mean_matched_errors, var_matched_errors


def process_solver_iterations(scan_matcher_df):
    if x_axis == 'translation':
        initial_errors = np.sqrt(np.square(scan_matcher_df.initial_error_x.as_matrix())
                                 + np.square(scan_matcher_df.initial_error_y.as_matrix())
                                 + np.square(scan_matcher_df.initial_error_z.as_matrix()))
    else:
        initial_errors = scan_matcher_df.initial_error_angle.as_matrix()
    initial_errors = np.reshape(initial_errors, (-1, batch_size))
    times = scan_matcher_df.solver_iterations.as_matrix()
    times = np.reshape(times, (-1, batch_size))
    mean_initial_errors = np.mean(initial_errors, 1)
    mean_times = np.mean(times, 1)
    var_times = np.std(times, 1)
    return mean_initial_errors, mean_times, var_times


def process_rotation_error(scan_matcher_df):

    if x_axis == 'translation':
        initial_errors = np.sqrt(np.square(scan_matcher_df.initial_error_x.as_matrix())
                                 + np.square(scan_matcher_df.initial_error_y.as_matrix())
                                 + np.square(scan_matcher_df.initial_error_z.as_matrix()))
    else:
        initial_errors = scan_matcher_df.initial_error_angle.as_matrix()
    initial_errors = np.reshape(initial_errors, (-1, batch_size))
    errors = scan_matcher_df.matched_error_angle.as_matrix()
    errors = np.reshape(errors, (-1, batch_size))
    mean_initial_errors = np.mean(initial_errors, 1)
    mean_times = np.mean(errors, 1)
    var_times = np.std(errors, 1)
    return mean_initial_errors, mean_times, var_times


def plot_generic(df, scan_matchers, processing_function, ylabel, title):
    processed_error_data = []
    n_sm = len(scan_matchers)
    for scan_matcher in scan_matchers:
        processed_error_data.append(processing_function(df[df.scan_matcher == scan_matcher]))
    N = processed_error_data[0][0].size
    ind = np.arange(N)  # the x locations for the groups
    width = 0.8 / n_sm  # the width of the bars
    fig, ax = plt.subplots()
    bars = []
    bar_index = 0
    for data in processed_error_data:
        plt.fill_between(data[0], data[1]-2*data[2], data[1]+2*data[2], color=colors_pastel[bar_index], alpha=0.5)
        bar_index += 1

    bar_index = 0
    for data in processed_error_data:
        bars.append(ax.plot(data[0], data[1], color=colors[bar_index]))
        bar_index += 1

    ax.set_ylabel(ylabel)
    ax.set_xlabel('Initial Error')
    plt.title(title)

    legend_handles = []
    for bar in bars:
        legend_handles.append(bar[0])
    ax.legend(legend_handles, scan_matchers)


local_path = "/thesis/scan_benchmark/scan_matching_benchmark_10-01-2017_19-06-07.csv"
df = pd.read_csv(os.path.expanduser('~') + local_path)
scan_matchers = df.scan_matcher.unique()
print(df.scan_matcher.unique())
batch_size = int((df.initial_error_x == 0).astype(int).sum() / scan_matchers.size)
print(batch_size)
x_axis = 'rotation'
for boundary_extrapolation in [True, False]:
    for cubic_interpolation in [True, False]:
        filtered_df = df[df.boundary_extrapolation == boundary_extrapolation]
        filtered_df = filtered_df[filtered_df.cubic_interpolation == cubic_interpolation]
        batch_size = 2 #int((filtered_df.initial_error_x == 0).astype(int).sum() / scan_matchers.size)
        boundary_title = ' with boundary' if boundary_extrapolation == 1 else ' without boundary'
        interpolation_title = ' with cubic_interpolation' if cubic_interpolation == 1 else ' with linear_interpolation'

        #plot_generic(filtered_df, scan_matchers, process_matching_error, 'Matching Error', 'Matching Error' + boundary_title + interpolation_title)
        #plot_generic(filtered_df, scan_matchers, process_solver_iterations, 'Solver Iterations', 'Solver Iterations' + boundary_title + interpolation_title)
        plot_generic(filtered_df, scan_matchers, process_rotation_error, 'Rotation Error', 'Rotation Error' + boundary_title + interpolation_title)
        #plot_generic(filtered_df, scan_matchers, process_matching_time, 'Matching time', 'Matching time' + boundary_title + interpolation_title)
        #plot_generic(filtered_df, scan_matchers, process_map_update_time, 'Map Update time', 'Map Update time' + boundary_title + interpolation_title)

plt.show()