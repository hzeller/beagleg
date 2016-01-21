all : machine-control.png print-stats.png

 %.svg : %.dot
	dot -Tsvg $^ > $@

 %.png : %.dot
	dot -Tpng $^ > $@
