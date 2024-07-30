
set title "plot_objfunc_shape"

# set xlabel "sigbase_x"
# set ylabel "sigbase_y"
# set grid
# set size ratio 1
# set xrange [-50:50]
# plot \
#     "data/data_objfunc.dat" with lines

set xlabel "y_base"
set ylabel "x_gear"
set zlabel "g(y_base, x_gear)"
set xrange [-50:50]
set yrange [-50:50]
# set dgrid3d 30, 30
set hidden3d
splot \
    # "data/data_objfunc.dat" using 1:2:3 with linespoints palette title "g() shape"
    # "data/data_objfunc.dat" using 1:2:3 with lines palette title "g() shape"
    "data/data_objfunc.dat" using 1:2:3 with lines


while (1) {
    print"interactive mode ('q' to quit): "
    val = real(STDIN)
    if (val == NaN) break

    replot
}
