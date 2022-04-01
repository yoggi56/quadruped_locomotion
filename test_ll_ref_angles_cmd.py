import lcm
from msgs import ref_angles
import time

ll_cmd_channel = "LL_COMMAND"

def handler(channel, data):
	msg = ref_angles.decode(data)
	print("I got {0}: ".format(msg.angle))
    

lc = lcm.LCM()
subscr = lc.subscribe(ll_cmd_channel, handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass
