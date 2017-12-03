import shapely.geometry
import shapely.prepared
import numpy as np
import matplotlib.pyplot as plt

n = 100
list_of_points = [None]*n*n
for i in range(n):
    for j in range(n):
        list_of_points[i*n+j] = shapely.geometry.Point((i,j))
polygon = shapely.geometry.Point(25, 50).buffer(20)
prepared_polygon = shapely.prepared.prep(polygon)
interior = map(prepared_polygon.contains, list_of_points)
occupancy = np.array(interior, dtype=np.bool)
occupancy = occupancy.reshape((n,n))
print(occupancy)
plt.imshow(occupancy)
plt.show()
