#!/bin/sh

PANDOC_OPTS=""

find ./ -iname "*.md" -exec sh -c 'pandoc ${0} --pdf-engine=xelatex -V geometry:margin=0.5in -o "${0%.md}.pdf"' {} \;

