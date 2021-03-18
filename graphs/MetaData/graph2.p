# Gnuplot script file for plotting data in file "LQ-lfemur-X-F600-F800-N20.dat", "BQ-lfemur-X-F600-F800-N20.dat"
# This file is called   graph2.p
set   autoscale                        # scale axes automatically
unset log                              # remove any log-scaling
unset label                            # remove any previous labels
set xtic 20 nomirror                            # set xtics automatically
set ytic auto nomirror                         # set ytics automatically
set title "Graph #2 SLERP quaternion vs Bezier SLERP interpolation" font ",12"
set xlabel "Frame #" font ",12"
set ylabel "Rotation around X axis (degree)" font ",12"
set key font ",12"
set xr [600:800]
set yr [-50:-10]
plot    "LQ-lfemur-X-F600-F800-N20.dat" using 1:2 t 'input' w lines lw 2 lt rgb "#F44336", \
     	"LQ-lfemur-X-F600-F800-N20.dat" using 1:3 t 'SLERP quaternion' w lines lw 2 lt rgb "#8CC24A", \
	"BQ-lfemur-X-F600-F800-N20.dat" using 1:3 t 'Bezier SLERP quaternion' w lines lw 2 lt rgb "#039BE5"
