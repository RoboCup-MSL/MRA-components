#!/bin/bash
INPUT=$1
OUTPUT=$2

echo '<?xml version="1.0" encoding="utf-8"?>' > $OUTPUT
echo '<tns:Situation xmlns:tns="http://www.robotsports.nl" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://www.robotsports.nl StrategyTester.xsd">' >> $OUTPUT
echo '<tns:Description>created from file: '$INPUT'</tns:Description>' >> $OUTPUT
echo '<tns:Options svgOutputFileName="created_'$INPUT'"/>' >> $OUTPUT
echo '<tns:Simulation numberOfIterations="1" simulateOpponent="false"/>' >> $OUTPUT
grep tns $INPUT >> $OUTPUT
echo '</tns:Situation>' >> $OUTPUT
