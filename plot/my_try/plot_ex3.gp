
# multiplot
x_shift = 5
y_shift = 3

f1(x) = (x)**2

set multiplot layout 1, 2 title "Coordinate Systems" font ",14"

set title "Coordinate System 1"
set xlabel "x"
set ylabel "f1(x)"
set grid
plot [-10:10] f1(x) with lines title "y = x^2"

set title "Absolute Coordinate System"
set xlabel "X"
set ylabel "Y"
set grid
f_absolute(x) = (x - x_shift)**2 + y_shift
plot [-5:15] f_absolute(x) with lines title "y = (x - 5)^2 + 3"

unset multiplot
