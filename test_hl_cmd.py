print('__file__={0:<35} | __name__={1:<20} | __package__={2:<20}'.format(__file__,__name__,str(__package__)))
import lcm

from msgs import hl_command_msg

def hndlr(channel, data):
    pos = [0]*12
    vel = [0]*12
    torq = [0]*12
    kp = [0]*12
    kd = [0]*12
    msg = hl_command_msg.decode(data)
    print("Received message:")
    for i in range(12):
        pos[i] = msg.act[i].position
        vel[i] = msg.act[i].velocity
        torq[i] = msg.act[i].torque
        kp[i] = msg.act[i].kp
        kd[i] = msg.act[i].kd

    print("Position: ", pos)
    print("Velocity: ", vel)
    print("Torque: ", torq)
    print("Kp: ", kp)
    print("Kd: ", kd)

if __name__ == '__main__':
    hl_cmd_channel = "HL_COMMAND"
    lc = lcm.LCM()
    subscription = lc.subscribe(hl_cmd_channel, hndlr)

    while True:
        lc.handle()
