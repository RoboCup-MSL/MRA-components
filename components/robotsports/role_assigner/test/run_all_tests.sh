#!/usr/bin/bash
echo clearing temp files
rm -rf ./output-team
echo excuting script: "$0"
echo Teamplanner scripts
for filename in ../testdata/xml-inputfiles/*.xml; do
	./xmltester.sh "$filename"
done
