all : machine-control.svg print-stats.svg

machine-control.svg:
print-stats.svg:

 %.svg : %.dot
	dot -Tsvg $^ > $@
