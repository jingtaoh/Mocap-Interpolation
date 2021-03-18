# Gnuplot script file for plotting data in file "LE-root-Z-F200-F500-N20.dat", "LQ-root-Z-F200-F500-N20.dat"
# This file is called   graph1.p
set   autoscale                        # scale axes automatically
unset log                              # remove any log-scaling
unset label                            # remove any previous labels
set xtic 20 nomirror                          # set xtics automatically
set ytic 20 nomirror                          # set ytics automatically
set title "Graph #3 linear Euler vs SLERP quaternion interpolation" font ",12"
set xlabel "Frame #" font ",12"
set ylabel "Rotation around Z axis (degree)" font ",12"
set key font ",12"
set xr [200:500]
set yr [-140:60]
plot    "LE-root-Z-F200-F500-N20.dat" using 1:2 t 'input' w lines lw 2 lt rgb "#F44336", \
     	"LE-root-Z-F200-F500-N20.dat" using 1:3 t 'linear Euler' w lines lw 2 lt rgb "#8CC24A", \
	"LQ-root-Z-F200-F500-N20.dat" using 1:3 t 'SLERP quaternion' w lines lw 2 lt rgb "#039BE5"
