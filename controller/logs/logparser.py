__author__ = 'Kevin-Patxi'


import matplotlib.pyplot as plt
import json
with open("experiment_history.log",'r') as f:

    loc_x = []
    loc_y = []
    loc_z = []
    loc_yaw =[]
    loc_pitch =[]
    loc_roll =[]
    roll= []
    pitch = []
    yaw = []
    thrust = []
    in_out_toggle = 0
    i=0

    for line in f:
            if ("input" in line)&(not(in_out_toggle)):
                in_out_toggle = 1
                parts = json.loads(line)
                loc_x.append(float(parts['pos_x']))
                loc_y.append(float(parts['pos_y']))
                loc_z.append(float(parts['pos_z']))
                loc_yaw.append(float(parts['pos_yaw']))
                loc_roll.append(float(parts['pos_roll']))
                loc_pitch.append(float(parts['pos_pitch']))

            if ("output" in line) &(in_out_toggle):
                in_out_toggle = 0
                parts = json.loads(line)
                roll.append(float(parts['roll_out']))
                pitch.append(float(parts['pitch_out']))
                yaw.append(float(parts['yaw_out']))
                thrust.append(float(parts['thrust']))

                i = i+1
    f.close()
    plt.figure(1)
    plt.subplot(211)
    plt.plot(thrust)
    plt.ylabel('thrust')
    plt.subplot(212)
    plt.plot(loc_z)
    plt.axhline(.5)
    plt.ylabel('Real Z')

    plt.figure(2)
    plt.subplot(211)
    plt.plot(roll)
    plt.ylabel('roll')
    plt.subplot(212)
    plt.plot(loc_roll)
    plt.axhline(0)
    plt.ylabel('real roll')


    plt.figure(3)
    plt.subplot(211)
    plt.plot(pitch)
    plt.ylabel('pitch')
    plt.subplot(212)
    plt.plot(loc_pitch)
    plt.ylabel('real pitch')


    plt.figure(4)
    plt.subplot(211)
    plt.plot(yaw)
    plt.ylabel('yaw')
    plt.subplot(212)
    plt.plot(loc_yaw)
    plt.axhline(0)
    plt.ylabel('real yaw')


    plt.figure(5)
    plt.subplot(211)
    plt.plot(loc_x)
    plt.ylabel('x')
    plt.axhline(0)
    plt.subplot(212)
    plt.plot(loc_y)
    plt.axhline(0)
    plt.ylabel('y')


    plt.figure(6)
    plt.plot(loc_x[-2000:],loc_y[-2000:])
    plt.axhline(0)
    plt.axvline(0)
    plt.ylabel('Y')
    plt.xlabel('X')


    plt.show()


    print i





