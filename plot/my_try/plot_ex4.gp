
set grid
set title "Plot of f(x) = a * x"
set xlabel "x"
set ylabel "f(x)"

# parameter
a = 2
f(x) = a * x

# plot
plot f(x) with lines title sprintf("f(x) = %d * x", a)

# interactive communication
while (1) {
    print "Enter new value for a (or 'q' to quit): "
    input = system("read -p '' a; echo $a")
    if (strstrt(input, 'q') >= 0) break
    a = real(input)
    if (a == NaN) break
    replot f(x) with lines title sprintf("f(x) = %d * x", a)
}


