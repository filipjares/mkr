# Plot the maps using gnuplot

plot "data/map_from_the_paper-vroni_output.txt" i 0 w l lw 3, "" i 1 w l, "data/map_from_the_paper-mat.txt" using 1:2

plot "data/map_from_the_paper-ma.txt" using 2:3 w p, "" using 2:3:4 w circle
