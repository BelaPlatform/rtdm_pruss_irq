#!/bin/bash -x
# Monitor the xenomai and linux interrupts
COMMAND="cat /proc/xenomai/irq /proc/xenomai/sched/stat /proc/interrupts | grep \"pruss_evt\|rtdm\""
if [ "$1" = -w ]
then
	watch -n 0.4 "$COMMAND"
else
	bash -c "$COMMAND"
fi

