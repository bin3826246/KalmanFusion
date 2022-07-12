import matplotlib.pyplot as plt
import numpy as np

out_file = open('output/out-2.txt', 'r')
gt_file = open('output/gt-2.txt', 'r')

out_x = np.array([])
out_y = np.array([])
gt_x = np.array([])
gt_y = np.array([])


for line in out_file.readlines():
    line = line.strip('\n')
    data = line.split('\t')
    out_x = np.append(out_x, float(data[0]))
    out_y = np.append(out_y, float(data[1]))

count = 0
for line in gt_file.readlines():
    line = line.strip('\n')
    data = line.split('\t')
    gt_x = np.append(gt_x, float(data[0]))
    gt_y = np.append(gt_y, float(data[1]))
    count += 1
    if count > 900:
        break

plt.figure(figsize=(12, 8))
plt.plot(out_x, out_y, 'o', label='output')
plt.plot(gt_x, gt_y, '-', label='ground truth')

plt.legend()
plt.show()
