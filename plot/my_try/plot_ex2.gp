
# set parameter
a = 3

# set figure title & label title
set title "plot_ex2.gp"
set xlabel "x"
set ylabel "gear(x, a)"


# set function to be plotted
gear(x, a) = x**2 * a

plot "data.dat" using 1:(gear($1, a)) with lines title sprintf("a=%d", a)

while (1) {
    print "Enter new value for a (or 'q' to quit): "
    a = real(STDIN)
    if (a == NaN) break
    replot
}


