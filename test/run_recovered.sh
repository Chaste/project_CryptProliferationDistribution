#!/bin/bash
#calc(){ awk "BEGIN { print $* }"  }
#
# Script to illustrate running batch jobs and passing in arguments.
#
# 
# This script assumes that the following has been run successfully:
# scons co=1 b=GccOpt ts=projects/CryptProliferation2013/test/TestRecoveredCryptLiteratePaper.hpp
#

num_sweeps=20;

for (( i=0 ; i<${num_sweeps}+1 ; i++))
do
#mutant_prob=$(((i)/num_sweeps)) ; printf %d.%d  ${r%??} ${r#${r%??}}
mutant_prob=`echo "$i / $num_sweeps " | bc -l`

	#mutant_prob=calc $i/$num_sweeps;
	echo "i = " $i
	echo "mutant prob= " $mutant_prob
	# NB "nice -20" gives the jobs low priority (good if they are going to dominate the server and no slower if nothing else is going on)
	# ">" directs std::cout to the file.
	# "2>&1" directs std::cerr to the same place.
	# "&" on the end lets the script carry on and not wait until this has finished.
	nice -20 ../build/optimised/TestRecoveredCryptLiteratePaperRunner -MutantProb $mutant_prob > output/Recovered_${i}_Output.txt 2>&1 &
done

echo "Jobs submitted"