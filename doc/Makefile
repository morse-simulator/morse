VERSION=0.1
GUIDE_TITLE=User Guide
#GUIDE_TITLE=Developers Guide
BG_TEXT=DRAFT-v.$(VERSION)


GUIDE_TITLE_PROTECTED=$(shell echo $(GUIDE_TITLE) | sed 's/ /_/g' | tr A-Z a-z)

CONTENT=$(GUIDE_TITLE_PROTECTED)_content.tex
BLANK=blank.pdf
COVER=cover.svg.in

PDF=morse_$(GUIDE_TITLE_PROTECTED)-$(VERSION).pdf

CONTENTpdf=$(CONTENT:.tex=.pdf)

COVERsvg=$(COVER:.svg.in=.svg)
COVERpdf=$(COVERsvg:.svg=.pdf)

all: $(PDF)

$(PDF): $(CONTENTpdf) $(BLANK) $(COVERpdf)
	pdftk $(COVERpdf) $(BLANK) $(CONTENTpdf) cat output $(@)

$(CONTENTpdf): $(CONTENT)
	sed 's/@BGTEXT@/$(BG_TEXT)/g' $(<) > $(<)-intermediate
	sed -i 's/@DATE@/\\today/g' $(<)-intermediate
	pdflatex $(<)-intermediate
	pdflatex $(<)-intermediate

$(COVERpdf): $(COVERsvg)  
	inkscape --export-pdf $(@) $(<)

$(COVERsvg): $(COVER)
	sed 's/@VERSION@/$(VERSION)/g' $(<) > $(@)-intermediate
	sed 's/@GUIDE_TITLE@/$(GUIDE_TITLE)/g' $(@)-intermediate > $(@)

clean:
	rm -f *.aux *.log *.snm *.out *.toc *.nav *intermediate *~ $(COVERsvg) $(COVERpdf) $(CONTENTpdf)

distclean: clean
	rm -f $(PDF)
