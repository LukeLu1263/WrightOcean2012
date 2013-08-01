#!/bin/bash
TMP_DIR=$(mktemp -d)
$1 $2 $3 $TMP_DIR
for i in $TMP_DIR/*; do
  cmp -s $i $4/`basename $i` || (mv $i $4/`basename $i` && echo "`basename $i` (bhbsc-cmp-update)")
done
rm -r $TMP_DIR
