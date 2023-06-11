all : machine-control.svg print-stats.svg

 %.svg : %.dot
	dot -Tsvg $^ > $@

 %.png : %.dot
	dot -Tpng $^ > $@

 %.svg : %.dot
	dot -Tsvg $^ > $@
