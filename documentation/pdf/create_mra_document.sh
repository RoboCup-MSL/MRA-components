#!/bin/bash -x
asciidoctor-pdf -r asciidoctor-diagram --theme=./mra_theme.yml -a imagesdir=`pwd`/../modules/ROOT/images/ ../modules/ROOT/pages/index.adoc -o MSL_reference_architecture.pdf


