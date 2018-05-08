; A spiral cut in the YZ plane so that it shows
; up neatly as sine as watchde from the XY plane
#r=50
G19
F[150 * 60]
G1 X0 Y#r Z0
G2 Y#r Z0 I0 K#r X25
G2 Y#r Z0 I0 K#r X50
G2 Y#r Z0 I0 K#r X75
G2 Y#r Z0 I0 K#r X100
