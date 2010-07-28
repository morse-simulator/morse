VERSION=0.1
GUIDE_TITLE=User Guide
#GUIDE_TITLE=Developers Guide
BG_TEXT=v.$(VERSION)


GUIDE_TITLE_PROTECTED=$(shell echo $(GUIDE_TITLE) | sed 's/ /_/g' | tr A-Z a-z)

CONTENT=$(GUIDE_TITLE_PROTECTED).tex
COVER=cover.svg.in

PDF=morse_$(GUIDE_TITLE_PROTECTED)-$(VERSION).pdf

CONTENTpdf=$(CONTENT:.tex=.pdf)

COVERsvg=$(COVER:.svg.in=.svg)
COVERpdf=$(COVERsvg:.svg=.pdf)

all: $(PDF)

$(PDF): $(CONTENT) $(COVERpdf)

	sed 's/@BGTEXT@/$(BG_TEXT)/g' $(<) > $(PDF:.pdf=.tex)
	sed -i 's/@DATE@/\\today/g' $(PDF:.pdf=.tex)
	pdflatex $(PDF:.pdf=.tex)
	pdflatex $(PDF:.pdf=.tex)

$(COVERpdf): $(COVERsvg)  
	inkscape --export-pdf $(@) $(<)

$(COVERsvg): $(COVER)
	sed 's/@VERSION@/$(VERSION)/g' $(<) > $(@)-intermediate
	sed 's/@GUIDE_TITLE@/$(GUIDE_TITLE)/g' $(@)-intermediate > $(@)

clean:
	rm -f *.aux *.log *.snm *.out *.toc *.nav *intermediate *~ $(COVERsvg) $(COVERpdf) $(CONTENTpdf) $(PDF:.pdf=.tex)

distclean: clean
	rm -f $(PDF)
