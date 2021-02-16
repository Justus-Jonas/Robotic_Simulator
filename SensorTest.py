import Sensor
import numpy as np


if __name__ == '__main__':
    map = a = np.full((100, 100), None)
    compute = Sensor.SensorComputation(0,map,4)
    list_of_distances = compute.update_distance(0,map,50,80)
    for x in list_of_distances:
        print(x.currentAngle, ':  ', x.distance)

