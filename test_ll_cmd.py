import lcm
from msgs import velocity
import time

ll_cmd_channel = "LL_COMMAND"

msg = velocity()
lc = lcm.LCM()

for i in range(20):
    msg.vx = i*10
    msg.vy = i*5
    msg.wz = i
    lc.publish(ll_cmd_channel, msg.encode())
    print("Data transmitted:")
    print("    vx = ", msg.vx)
    print("    vy = ", msg.vy)
    print("    wz = ", msg.wz)
    time.sleep(2)
