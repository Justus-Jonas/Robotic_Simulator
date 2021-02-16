import Sensor
import numpy as np
from timeit import default_timer as timer

if __name__ == '__main__':
    map = a = np.full((1001, 1001), None)
    compute = Sensor.SensorComputation(0,map,4)
    start = timer()
    list_of_distances = compute.update_distance(0,map,500,800)
    end = timer()
    print(end-start)
    for x in list_of_distances:
        print(x.currentAngle, ':  ', x.distance)

