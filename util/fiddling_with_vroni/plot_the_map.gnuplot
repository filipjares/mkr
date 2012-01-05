# Plot the maps using gnuplot

plot "data/map_from_the_paper-vroni_output.txt" i 0 w l lw 3, "" i 1 w l, "data/map_from_the_paper-mat.txt" using 1:2
