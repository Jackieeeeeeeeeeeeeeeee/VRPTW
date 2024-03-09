import cvrplib
import numpy as np
from scip_routing.solver import VRPTWSolver
from scip_routing.utils import instance_graph
import math
import time

# def euclidean_distance(point1, point2):
#     return round(math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2), 2)

# def calculate_distances(coordinates):
#     num_coordinates = len(coordinates)
#     distances = [[0.0] * num_coordinates for _ in range(num_coordinates)]

#     for i in range(num_coordinates):
#         for j in range(num_coordinates):
#             distances[i][j] = euclidean_distance(coordinates[i], coordinates[j])

#     return distances

instance, _ = cvrplib.download('C103', solution=True)
instance.dimension = 26
instance.n_customers = 25
instance.customers = instance.customers[0:25]
instance.coordinates = instance.coordinates[0:26]
instance.demands = instance.demands[0:26]
instance.service_times = instance.service_times[0:26]
instance.latest = instance.latest[0:26]
instance.earliest = instance.earliest[0:26]
points = instance.coordinates
sub_distances = [row[0:26] for row in instance.distances[0:26]]
#sub_distances = calculate_distances(points)
instance.distances = sub_distances

instance_graph = instance_graph(instance)



solver = VRPTWSolver(
        graph=instance_graph,
        instance=instance,
        verbosity=2,
        pricing_strategy="py", # "py" also can be used for the pure-python pricer
    )

time_start = time.perf_counter()  # 记录开始时间

solver.solve()


time_end = time.perf_counter()  # 记录结束时间
time_sum = time_end - time_start  # 计算的时间差为程序的执行时间，单位为秒/s
print(time_sum)