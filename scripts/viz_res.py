#!/usr/bin/env python
import sys
import matplotlib.pyplot as plt

pointsf = sys.argv[1]
edgesf = sys.argv[2]
removed_edgesf = sys.argv[3]

points = []
num_clusters = 0
clusters = []
edges = []
removed_edges = []

with open(pointsf) as f:
    cnt = 0
    for line in f:
        if cnt == 0:
            num_clusters = int(line)
            cnt += 1
            continue
        line = line.strip().split(",")
        x = float(line[0])
        y = float(line[1])
        cluster = int(line[2])
        points.append((x,y))
        clusters.append(cluster)

with open(edgesf) as f:
    for line in f:
        line = line.split(" ")
        x = int(line[0])
        y = int(line[1])
        edges.append((x,y))

with open(removed_edgesf) as f:
    for line in f:
        line = line.split(" ")
        x = int(line[0])
        y = int(line[1])
        removed_edges.append((x,y))

fig, ax = plt.subplots()

hsv = plt.get_cmap('hsv')
for i in xrange(len(points)):
    x,y = points[i]
    c = hsv(float(clusters[i])/num_clusters)
    ax.scatter(x, y, c=c)

plt.show()
