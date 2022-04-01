import lcm
from msgs import measurment
import time


hl_state_channel = "HL_STATE"

msg = measurment()
lc = lcm.LCM()

for i in range(20):
    msg.imu.euler.x = i*10
    msg.imu.euler.y = i*5
    msg.imu.euler.z = i
    msg.act[0].position = i*0.5
    lc.publish(hl_state_channel, msg.encode())
    print("Data transmitted:")
    print("    e_x   = ", msg.imu.euler.x)
    print("    e_y   = ", msg.imu.euler.y)
    print("    e_z   = ", msg.imu.euler.z)
    print("    pos_0 = ", msg.act[0].position)
    time.sleep(2)
