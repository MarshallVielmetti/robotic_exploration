import numpy as np
from collections import deque


def get_esdf(map):
    esdf = np.zeros(map.shape)
    esdf.fill(np.inf)

    queue = deque()

    # Initialize the queue with the known obstacles
    for i, j in zip(*np.where(map == 100)):
        esdf[i, j] = 0
        queue.append((i, j))

    while queue:
        i, j = queue.popleft()

        for i_, j_ in [(i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)]:
            if i_ < 0 or i_ >= map.shape[0] or j_ < 0 or j_ >= map.shape[1]:
                continue

            if map[i_, j_] == 100:
                continue

            if esdf[i_, j_] != np.inf:
                esdf[i_, j_] = min(esdf[i_, j_], esdf[i, j] + 1)
            else:
                esdf[i_, j_] = esdf[i, j] + 1
                queue.append((i_, j_))

    return esdf
