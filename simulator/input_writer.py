#!/usr/bin/env python3

import csv
import math
import sys

mode = 'w'
mode_i = sys.argv[1]
# mode_i = '0'

print(mode_i)
with open('/home/sota/rocker-bogie-simulation/simulator/input.csv', mode) as csvfile:
    writer = csv.writer(csvfile, lineterminator='\n')
    if(mode == 'w'):
        writer.writerow(['時刻[sec]','リンク関節制御モード(トルク制御:1　角度制御:1)','前輪の入力トルク[N m]','中輪の入力トルク[N m]','後輪の入力トルク[N m]','リンク関節への入力[N m]|mode=0 or [deg]|mode=1]'])
    elif(mode == 'a'):
        csvfile.write('\n')

    time = 0
    control_mode = 4
    period =- 1/1

    ft = 0
    mt = 0
    rt = 0
    jt = 5
    itr = 100000    
    ang_ini = 23.1766

    p= 'e'

    # for i in range(itr):
    #     jt_sin = -30  + jt * math.sin(2*math.pi/period * i*0.001)
    #     writer.writerow(['{:.3f}'.format(i*0.001), control_mode, ft, mt, rt, jt_sin])
    
    if(mode_i == '0'):
         writer.writerow([0, 3, 0, 0, 0, 0])
         writer.writerow([1, 3, 0, 0, 0, 0])
    else:
        for i in range(itr):
            jt_sin = ang_ini + -0.005*i  + jt * math.sin(2*math.pi/period * i*0.001)
            writer.writerow(['{:.3f}'.format(i*0.001), control_mode, ft, mt, rt, jt_sin])
    
    # itr2 = 1000
    # for i in range(itr2):
    #     jt_input = -40 * i/itr2 
    #     if(i > itr2):
    #         jt_input = -40
    #     writer.writerow(['{:.3f}'.format(i*0.001), control_mode, ft, mt, rt, jt_input])


    # for i in range(itr):
    #     writer.writerow(['{:.3f}'.format(time), control_mode, ft, mt, rt, jt])
    #     time += period/2
    #     if(p == 'e'):
    #         jt *= -1
    