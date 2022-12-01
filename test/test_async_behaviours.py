from time import sleep

import rclpy

from python_interface.drone_interface import DroneInterface

rclpy.init()

drone_interface = DroneInterface("drone_sim_0", verbose=False, sync=False)
drone_interface.arm()
drone_interface.offboard()

drone_interface.takeoff(3, 2)
print("Takeoff completed\n")
sleep(1)

goto = drone_interface.go_to(0, 30, 1, 0.5)

print("Goto async")
print(goto.status)
sleep(1.0)
print(goto.status)
sleep(0.5)
goto.stop()
print(goto.status)


print("Path finished")

drone_interface.land(0.2)
drone_interface.shutdown()

print("Bye!")
