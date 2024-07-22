
# all from gnuplot documentation
# https://www.cs.hmc.edu/~vrable/gnuplot/using-gnuplot.html



plot [-4:4] exp(-x**2 / 2)

plot [t=-4:4] exp(-t**2 / 2), t**2 / 16

f(x) = exp(-x**2 / 2)
plot [t=-4:4] f(t), t**2 / 16

plot [t=-4:4] f(t) title "Bell Curve", t**2 / 16 title "Parabola"



set title "Some Sample Plots"
set xlabel "Independent Variable (no units)"
set ylabel "Dependent Variable (no units)"
set grid

replot





