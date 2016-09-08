G90 G17

G1 F[150 * 60]

G1 X10 Y10

; Cubic splines
G5 I0 J60 P0 Q-60 X30 Y30	(a curvy N)
G5 P0 Q-60 X50 Y50		(another curvy N, attached smoothly)
G5 P0 Q-60 X70 Y70		(another curvy N, attached smoothly)
G5 P0 Q-60 X90 Y90		(another curvy N, attached smoothly)

G1 X100 Y100

; Quadratic spline
G5.1 X0 I-50 J-150		(a parabola)
