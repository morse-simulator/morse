VERSION=0.1
BG_TEXT=DRAFT-v.$(VERSION)

CONTENT=user_guide_content.tex
BLANK=blank.pdf
COVER=cover.svg.in

PDF=user_guide.pdf

CONTENTpdf=$(CONTENT:.tex=.pdf)

COVERsvg=$(COVER:.svg.in=.svg)
COVERpdf=$(COVERsvg:.svg=.pdf)

all: $(PDF)

$(PDF): $(CONTENTpdf) $(BLANK) $(COVERpdf)
	pdftk $(COVERpdf) $(BLANK) $(CONTENTpdf) cat output $(@)

$(CONTENTpdf): $(CONTENT)
	sed 's/@BG_TEXT@/$(BG_TEXT)/g' $(<) > $(<)-intermediate
	pdflatex $(<)-intermediate
	pdflatex $(<)-intermediate

$(COVERpdf): $(COVERsvg)  
	inkscape --export-pdf $(@) $(<)

$(COVERsvg): $(COVER)
	sed 's/@VERSION@/$(VERSION)/g' $(<) > $(@)

clean:
	rm -f *.aux *.log *.snm *.out *.toc *.nav *intermediate *~ $(COVERsvg) $(COVERpdf) $(CONTENTpdf)

distclean: clean
	rm -f $(PDF)
