set datafile separator "\t"

file="/home/sota/rocker-bogie-simulation/simulator/simulation_log.csv"
set terminal png 
set grid
set xlabel "Time[sec]"
set ylabel "Force[N]"

set y2tics
set y2label "Motor torque[N m]"
set y2range [-1:1]


set terminal png
set output "/home/sota/rocker-bogie-simulation/simulator/log/images/whole.png"
set title "Excitation"
set ylabel "angle between bogie-link and motor-link [deg]"
set y2label "angle between rocker-link and bogie-link [deg]"
set autoscale y2
p [0:0.8] file u 1:17 axis x1y1 title "angle between bogie-link and motor-link" w l lw 1, file u 1:5 axis x1y1 title "angle between rocker-link and bogie-link" w l lw 1; 

set terminal png
set output "/home/sota/rocker-bogie-simulation/simulator/log/images/expantion.png"
set title ""
set ylabel "angle between bogie-link and motor-link [deg]"
set y2label "angle between rocker-link and bogie-link [deg]"
set autoscale y2
p [0:0.8] file u 1:17 axis x1y1 title "angle between bogie-link and motor-link" w l lw 1, file u 1:5 axis x1y1 title "angle between rocker-link and bogie-link" w l lw 1; 

set terminal png
set output "/home/sota/rocker-bogie-simulation/simulator/log/images/motorLink.png"
set title "Excitation"
set ylabel "angle between bogie-link and motor-link [deg]"
set y2label "angle between rocker-link and bogie-link [deg]"
set autoscale y2
p [] file u 1:17 axis x1y1 title "angle between bogie-link and motor-link" w l lw 1; 

# set terminal pdf enhanced
set output "/home/sota/rocker-bogie-simulation/simulator/log/images/reishin.png"
set title ""
set ylabel "Joint angle [deg]"
set y2label "Motor torque [N m]"
set yrange [-100:100]
set y2range [-1:1]
p [0:1.2][] file u 1:15 axis x1y1 title "angle between rocker-link and motor-link" w l lw 2,file u 1:5 axis x1y1 title "angle between rocker-link and bogie-link" w l lw 2 lc "light-red",file u 1:16 axis x1y2 title "motor torque" w l lw 2 lc 'sea-green'; 
# p [] file u 1:15 axis x1y1 title "angle between rocker-link and motor-link" w l lw 2,file u 1:5 axis x1y1 title "angle between rocker-link and bogie-link" w l lw 2 lc "light-red",file u 1:16 axis x1y2 title "motor torque" w l lw 2 lc 'sea-green'; 

set terminal png
set output "/home/sota/rocker-bogie-simulation/simulator/log/images/full.png"
set terminal pngcairo size 1280, 480
set title "{/Symbol q} = 23 + -0.005 t + 5 sin(2{/Symbol p} / 1 * 0.001t)   t: Time[ms]"
set ylabel "Angle [deg]"
set y2label ""
set y2label "Torque[N m]"
set y2range [-0.6:0.6]
# set autoscale y2
p [][-60:60] file u 1:17 axis x1y1 title "angle between bogie-link and motor-link" w l lw 1, file u 1:5 axis x1y1 title "angle between rocker-link and bogie-link" w l lw 1,file u 1:16 axis x1y2 title "Motor torque" w l lw 2; 

set output "/home/sota/rocker-bogie-simulation/simulator/log/images/M_Torque-M_angle.png"
set title "Motor Torque - Motor angle"
set ylabel "Joint angle [deg]"
set y2label "Motor torque[N m]"
set autoscale y2
p [][] file u 1:17 axis x1y1 title "angle between bogie-link and motor-link" w l lw 2,file u 1:14 axis x1y1 title "Target Angle" w l lw 2,file u 1:16 axis x1y2 title "Motor torque" w l lw 2

set terminal png
set output "/home/sota/rocker-bogie-simulation/simulator/log/images/Narrow.png"
set title "Narrow range, {/Symbol q}_{offset} = 49 [deg]"
set ylabel "Angle [deg]"
set y2label ""
set autoscale y2
p [][] file u 1:17 axis x1y1 title "angle between bogie-link and motor-link" w l lw 1, file u 1:5 axis x1y1 title "angle between rocker-link and bogie-link" w l lw 1, file u 1:($4 + $5) axis x1y1 title "Absolute angle of bogie-link" w l lw 1 lc "light-red"; 

set terminal png
set output "/home/sota/rocker-bogie-simulation/simulator/log/images/B_M_Link.png"
set title "Excitation"
set ylabel "Joint angle [deg]"
set yrange []
set y2range []
p [][] file u 1:17 axis x1y1 title "angle between rocker-link and motor-link" w l lw 2, 



set output "./images/Horizontal.png"
set title "Horizontal force"
set ylabel "force[N]"
p [0:1.2] file u 1:13 title "forward" w l lw 2,file u 1:14 title "middle" w l lw 2,file u 1:15 title "back" w l lw 2; 

set output "./images/image_f.png"
set title "Front Wheel"
p [0:1.2][-30:30]  file u 1:10 title "Horizontal Friction" w l lw 2,file u 1:13 w l lw 2 title "Vertical Friction";

set output "./images/image_m.png"
set title "Middle Wheel"
p [0:1.2][-30:30] file u 1:11 title "Horizontal Friction" w l lw 2,file u 1:14 w l lw 2 title "Vertical Friction";

set output "./images/image_b.png"
set title "back Wheel"
p [0:1.2][-30:30] file u 1:12 title "Horizontal Friction" w l lw 2,file u 1:15 w l lw 2 title "Vertical Friction";

set output "./images/thera_r.png"
set title "Posture of the rocker link"
set ylabel "Angle[rad]"
p [0:1.2] file u 1:4 w l lw 2;
  