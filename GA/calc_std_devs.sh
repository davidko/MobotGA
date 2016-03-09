#!/bin/bash
echo $#
if [ $# -ne 1 ] ; then
  echo "Command Format: $0 <logfile_dir>"
  exit
fi

logdir=`basename $1`

for gen in $logdir/gen*; do
  files=$gen/chrom*
  GA_diversity.py 40 $files
done
