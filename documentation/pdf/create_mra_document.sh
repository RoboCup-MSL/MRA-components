#!/bin/bash -x
# generate pdf from ascii doc pages. Required tool: asciidoctor-pdf. Install instructions on: docs.asciidoctor.org.
# use version 2.3.x or higher (use ruby install method for Ubuntu 20.04)
# install asciidoctor, asciidoctor-pdf, asciidoctor-kroki, asciidoctor-plantuml, asciidoctor-diagram
asciidoctor-pdf -r asciidoctor-diagram --theme=./mra_theme.yml -a imagesdir=`pwd`/../modules/ROOT/images/ ../modules/ROOT/pages/index.adoc -o MSL_reference_architecture.pdf


