
set title "plot_objfunc_shape"
set xlabel "sigbase_x"
set ylabel "sigbase_y"

set grid

set size ratio 1
set xrange [-50:50]


plot \
    "data/data_objfunc.dat" with lines



while (1) {
    print"interactive mode ('q' to quit): "
    val = real(STDIN)
    if (val == NaN) break

    replot
}
