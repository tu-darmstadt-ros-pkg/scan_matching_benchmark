import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import pandas as pd
import os as os
import numpy as np

def process_scan_matcher(scan_matcher_df):
    initial_errors = np.sqrt(np.square(scan_matcher_df.initial_error_x.as_matrix())
                             + np.square(scan_matcher_df.initial_error_y.as_matrix())
                             + np.square(scan_matcher_df.initial_error_z.as_matrix()))
    matched_errors = np.sqrt(np.square(scan_matcher_df.matched_error_x.as_matrix())
                             + np.square(scan_matcher_df.matched_error_y.as_matrix())
                             + np.square(scan_matcher_df.matched_error_z.as_matrix()))
    initial_errors = np.reshape(initial_errors, (-1, batch_size))
    matched_errors = np.reshape(matched_errors, (-1, batch_size))

    mean_initial_errors = np.mean(initial_errors, 1)
    mean_matched_errors = np.mean(matched_errors, 1)
    var_matched_errors = np.var(matched_errors, 1)
    return mean_initial_errors, mean_matched_errors, var_matched_errors

batch_size = 5
df = pd.read_csv(os.path.expanduser('~') + "/scan_matching_benchmark_09-14-2017_12-21-05.csv")
scan_matchers = ['ProbabilityGridScanMatcher', 'ChiselTSDFScanMatcher', 'VoxbloxTSDFScanMatcher',
                 'VoxbloxESDFScanMatcher']

pg_mean_initial_errors, pg_mean_matched_errors, pg_var_matched_errors = process_scan_matcher(df[df.scan_matcher ==
                                                                                       'ProbabilityGridScanMatcher'])
ct_mean_initial_errors, ct_mean_matched_errors, ct_var_matched_errors = process_scan_matcher(df[df.scan_matcher ==
                                                                                       'ChiselTSDFScanMatcher'])
vt_mean_initial_errors, vt_mean_matched_errors, vt_var_matched_errors = process_scan_matcher(df[df.scan_matcher ==
                                                                                       'VoxbloxTSDFScanMatcher'])
ve_mean_initial_errors, ve_mean_matched_errors, ve_var_matched_errors = process_scan_matcher(df[df.scan_matcher ==
                                                                                       'VoxbloxESDFScanMatcher'])

N = ve_mean_initial_errors.size
print(N)
ind = np.arange(N)  # the x locations for the groups
width = 0.2       # the width of the bars

fig, ax = plt.subplots()
rects1 = ax.bar(ind, pg_mean_matched_errors, width, color='r', yerr=pg_var_matched_errors)
rects2 = ax.bar(ind + width, ct_mean_matched_errors, width, color='y', yerr=ct_var_matched_errors)
rects3 = ax.bar(ind + width*2, vt_mean_matched_errors, width, color='g', yerr=vt_var_matched_errors)
rects4 = ax.bar(ind + width*3, ve_mean_matched_errors, width, color='b', yerr=ve_var_matched_errors)

# add some text for labels, title and axes ticks
ax.set_ylabel('Matching Error')
ax.set_xlabel('Initial Error')
ax.set_xticks(ind + width / 2)
ax.set_xticklabels(np.round(pg_mean_initial_errors, 2))
ax.set_ylim(ymax=0.001)


ax.legend((rects1[0], rects2[0], rects3[0], rects4[0]), scan_matchers)


plt.show()




#plt.legend()
plt.show()
