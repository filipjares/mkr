#!/bin/bash

# Sample input file is there in two versions:
#		data/map_from_the_paper.dxf
#		data/map_from_the_paper.svg
# It was drawn in QCad and inspired by the figure 2 in the article "Coordinated multi-robot
# exploration using a segmentation of the environment" by Wurm, Stachniss and Burgard

# MA output of the VRONI with comments included is in data/map_from_the_paper-ma-commented.txt
# corresponding illustration is in data/map_from_the_paper-ma_circles_and_comments_added.dxf

# Check Vroni and xml2txt are in PATH
if [ -z `which vroni` ]; then
	echo "vroni has to be in your PATH.";
	exit 1;
fi
if [ -z `which xml2txt` ]; then
	echo "xml2txt has to be in your PATH. It is used to convert Ipe output of Vroni to text file.";
	exit 1;
fi

VRONI_INPUT=data/map_from_the_paper.dxf
VRONI_OUTPUT=data/map_from_the_paper-vroni_output.xml
VRONI_TXT_OUTPUT=data/map_from_the_paper-vroni_output.txt
VRONI_MA_OUTPUT=data/map_from_the_paper-ma.txt

echo
echo "Vroni will be launched. It is interactive application. You have to press 'v'"
echo "and then 'q' in order to process the input, plot and save results and to quit"
echo "the Vroni program."
echo
echo "Press enter"
read

# expects vroni is in PATH; may want to use --X11 instead of --OGL
vroni --OGL --Ipe $VRONI_OUTPUT --mic --wmat --ma $VRONI_MA_OUTPUT --file $VRONI_INPUT

# expects xml2txt (a converter by Vojtech Vonasek) is in PATH
xml2txt $VRONI_OUTPUT $VRONI_TXT_OUTPUT

echo "================================================================================"
echo
echo "The Vroni output has been saved to '$VRONI_OUTPUT'."
echo "It is Ipe6 file containing both the input and output data in several layers."
echo "Border and voronoi vertices has been also saved in simple textual format which"
echo "can be processed with gnuplot to '$VRONI_TXT_OUTPUT'."
echo
echo "You can use following gnuplot command to plot it:"
echo
echo "plot \"$VRONI_TXT_OUTPUT\" i 0 w l lw 3, "" i 1 w l"
echo
echo "The MAT layer may be extracted from '$VRONI_OUTPUT'"
echo "by stripping the xml markup and trailing 'l's and 'm's. You obtain data in"
echo "following form (segments divided by empty lines):"
echo
cat <<END
# rucne ziskana data - MAT - z map_from_the_paper.xml:
0.000000 223.653806
111.826903 111.826903

394.669140 135.500464
394.661853 134.207764
394.653838 133.627164
394.645020 133.148602
394.635321 132.715603

337.338281 223.653806
256.006331 304.985755

228.045926 111.826903
111.826903 111.826903
END
echo
echo "If you save it to 'XXX', you can plot it using"
echo
echo "plot \"XXX\" using 1:2"
echo

