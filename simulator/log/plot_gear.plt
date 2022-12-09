set datafile separator "\t"

file="/home/kanno/gear_simulator/simulator/simulation_log.csv"
#[8:10] [8:10]
set terminal png
set output "/home/kanno/gear_simulator/simulator/log/images/position.png"
set title "Position"
set ylabel "Position [m]"
p [8:10] file u 1:2 axis x1y1 title "x[m]" w l lw 1, file u 1:3 axis x1y1 title "z[m]" w l lw 1; 

set terminal png
set output "/home/kanno/gear_simulator/simulator/log/images/high.png"
set title "High"
set ylabel "Position [m]"
p [8:10] file u 1:16 axis x1y1 title "zg[m]" w l lw 1, file u 1:3 axis x1y1 title "z[m]" w l lw 1; 

#p=plot u=using
set terminal png
set output "/home/kanno/gear_simulator/simulator/log/images/Angle.png"
set title "Angle"
set ylabel "Angle [deg]"
# set y2label "angle between rocker-link and bogie-link [deg]"
# set autoscale y2
p [8:10] file u 1:4 axis x1y1 title "angle 1[deg]" w l lw 1, file u 1:5 axis x1y1 title "angle m[deg]" w l lw 1, file u 1:6 axis x1y1 title "angle 2[deg]" w l lw 1, file u 1:7 axis x1y1 title "angle 3[deg]" w l lw 1; 

set terminal png
set output "/home/kanno/gear_simulator/simulator/log/images/Angle2_ref.png"
set title "Angle2_ref"
set ylabel "Angle [deg]"
p [8:10] file u 1:6 axis x1y1 title "moter angle[deg]" w l lw 1, file u 1:15 axis x1y1 title "moter angle ref[deg]" w l lw 1;

set terminal png
set output "/home/kanno/gear_simulator/simulator/log/images/Excitation.png"
set title "Excitation"
set ylabel "Angle [deg]"
set y2label "Torque [N m]"
set autoscale y2
#set y2tics -1,0.2
set ytics nomirror

p [8:10] file u 1:6 axis x1y1 title "moter angle[deg]" w l lw 1, file u 1:14 axis x1y1 title "knee angle[deg]" w l lw 1, file u 1:5 axis x1y1 title "spring angle[deg]" w l lw 1, file u 1:11 axis x1y2 title "moter torque[N m]" w l lw 1, file u 1:13 axis x1y2 title "spring torque[N m]" w l lw 1; 

set terminal png
set output "/home/kanno/gear_simulator/simulator/log/images/Excitation_torque.png"
set title "torque"
set ylabel "Torque [N m]"
set y2label "force [N]"
#set y2tics 0,10
set ytics nomirror
set autoscale y2
p [8:10]  file u 1:11 axis x1y1 title "moter torque[N m]" w l lw 1, file u 1:13 axis x1y1 title "spring torque[N m]" w l lw 1,file u 1:8 axis x1y2 title "reaction force[N]" w l lw 1; 

set terminal png
set output "/home/kanno/gear_simulator/simulator/log/images/Excitation_Angle.png"
set title "Angle"
set ylabel "Angle [deg]"
set y2label "force [N]"
#set y2tics 0,10
set ytics nomirror
set autoscale y2
p [8:10]  file u 1:6 axis x1y1 title "moter angle[deg]" w l lw 1, file u 1:14 axis x1y1 title "knee angle[deg]" w l lw 1, file u 1:5 axis x1y1 title "spring angle[deg]" w l lw 1,file u 1:8 axis x1y2 title "reaction force[N]" w l lw 1; 