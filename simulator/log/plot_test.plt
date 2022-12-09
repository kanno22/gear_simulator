set datafile separator "\t"

file="/home/kanno/gear_simulator/simulator/simulation_log.csv"

set terminal png
set output "/home/kanno/gear_simulator/simulator/log/images/position.png"
set title "Position"
set ylabel "Position [m]"
p [0:0.8] file u 1:2 axis x1y1 title "x[m]" w l lw 1, file u 1:3 axis x1y1 title "z[m]" w l lw 1; 
#p=plot u=using
set terminal png
set output "/home/kanno/gear_simulator/simulator/log/images/Angle.png"
set title "Angle"
set ylabel "Angle [deg]"
# set y2label "angle between rocker-link and bogie-link [deg]"
# set autoscale y2
p [0:0.8] file u 1:3 axis x1y1 title "angle 1[deg]" w l lw 1, file u 1:4 axis x1y1 title "angle m[deg]" w l lw 1, file u 1:5 axis x1y1 title "angle 2[deg]" w l lw 1, file u 1:6 axis x1y1 title "angle 3[deg]" w l lw 1; 
