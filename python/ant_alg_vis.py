import numpy as np
import matplotlib.pyplot as plt

data = []

with open("../logs/antColony.txt") as file:
  for line in file:
    data.append([float(x) for x in line.split()])

data = np.array(data)

group_size = 10  # количество муравьев в одной итерации

# Получаем количество полных групп
num_groups = data.shape[0] // group_size

# Усредняем по каждой группе
mean_current_path = data[:num_groups * group_size, 2].reshape(num_groups, group_size).mean(axis=1)
mean_x = data[:num_groups * group_size, 0].reshape(num_groups, group_size).mean(axis=1)

fig, axs = plt.subplots(2, 1)

axs[0].plot(data[:, 0], data[:, 3], label="Pheromones on best path", lw=1, c='b')
axs[1].plot(data[:, 0], data[:, 1], label="Best path", lw=5, c='r')
axs[1].plot(mean_x, mean_current_path, label="Mean Current path", lw=2, c='g')
# plt.plot(data[:, 0], data[:, 2], label="Current path", lw=2, c='g')
plt.grid()
plt.legend()
plt.show()
