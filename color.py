import time
import Drone.Autonomous.dist_calulator as dist_calulator


print(dist_calulator.distance([53.32055555555556, -1.7297222222222221], [53.31861111111111, -1.6997222222222223]))

start = time.time()
time.sleep(2)
end = time.time()

print((int)(end-start))