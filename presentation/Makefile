
all: prezentace.pdf
.PHONY: all clean

# vickrat se to pousti kvuli odkazum apod.
prezentace.pdf: prezentace.tex bibliography.bib
	pdfcslatex $<; bibtex prezentace; pdfcslatex $<; pdfcslatex $<

clean:
	rm -f prezentace.6.vrb prezentace.aux prezentace.log prezentace.nav prezentace.out prezentace.pdf prezentace.snm prezentace.toc
