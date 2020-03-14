#!/bin/bash
cloc  src/ lib/ include/ test/

count=`cloc  src/ lib/ include/ test/ | awk '{if ($1=="SUM:") print $5}'`
echo
echo "TOTAL Line Count: $count"

echo "`date` $count" >> codeLineCountHistory.dat
