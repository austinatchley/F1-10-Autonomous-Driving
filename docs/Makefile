.SUFFIXES:

SOURCES=$(wildcard *.md)

all: $(SOURCES:.md=.pdf)

%.pdf: %.md
	pandoc $< --pdf-engine=xelatex -V geometry:margin=0.5in -o "$@"

