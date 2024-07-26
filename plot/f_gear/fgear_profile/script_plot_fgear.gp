
set title "sigbase_f_gear"
set xlabel "sigbase_x"
set ylabel "sigbase_y"

set grid

# set size ratio 1/2
# set xrange [-30:30]
# set yrange [0:30]

set size ratio 1
set xrange [-30:30]
set yrange [-30:30]

# plot "data_fgear.dat" with lines
# plot "data_fgear.dat"

plot \
    "data/data_fgear_radius5_theta80.dat" with lines, \
    "data/data_fgear_radius5_theta85.dat" with lines, \
    "data/data_fgear_radius5_theta90.dat" with lines, \
    "data/data_fgear_radius5_theta95.dat" with lines, \
    "data/data_fgear_radius5_theta100.dat" with lines, \
    "data/data_fgear_radius5_theta105.dat" with lines, \
    "data/data_fgear_radius5_theta110.dat" with lines, \
    "data/data_fgear_radius5_theta115.dat" with lines, \
    "data/data_fgear_radius5_theta120.dat" with lines, \
    "data/data_fgear_radius4_theta80.dat" with lines, \
    "data/data_fgear_radius4_theta85.dat" with lines, \
    "data/data_fgear_radius4_theta90.dat" with lines, \
    "data/data_fgear_radius4_theta95.dat" with lines, \
    "data/data_fgear_radius4_theta100.dat" with lines, \
    "data/data_fgear_radius4_theta105.dat" with lines, \
    "data/data_fgear_radius4_theta110.dat" with lines, \
    "data/data_fgear_radius4_theta115.dat" with lines, \
    "data/data_fgear_radius4_theta120.dat" with lines

while (1) {
    print "interactive mode ('q' to quit): "
    val = real(STDIN)
    if (val == NaN) break
    replot
}





