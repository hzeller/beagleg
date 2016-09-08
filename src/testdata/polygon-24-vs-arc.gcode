#1=24       ; Polygon faces count

#2=[360/#1] ; Angle.
#3=[-#2/2]  ; rotation offset. Half a segment off so that we have vertical lines.

#4=50       ; polygon radius
#5=45       ; circle radius G2
#6=40       ; circle radius G3

G92 X[-#4] Y[-#4]    ; Put center of circle on convenient (0,0) coordinate.

G1 F[150 * 60]
#7=0        ; Loop counter
WHILE [#7 <= #1] DO
      X[#4 * cos[#7 * #2 + #3]] Y[#4 * sin[#7 * #2 + #3]]
      #7=[#7+1]
END

; Now compare that to an inner 'regular' circle.
G1 X#5 Y0
G2 X#5 Y0 I[-#5] J0

G1 X#6 Y0
G3 X#6 Y0 I[-#6] J0
