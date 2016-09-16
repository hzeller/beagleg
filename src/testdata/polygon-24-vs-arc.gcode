#faces=24       ; Polygon faces count
#angle=[360/#faces] ; Angle.
#aoff=[-#angle/2]  ; rotation offset. Half a segment off so that we have vertical lines.

#poly_r=50      ; polygon radius
#cg2_r=45       ; circle radius G2
#<cg3_r>=40     ; circle radius G3. Example with parameter bracket syntax.

G92 X[-#poly_r] Y[-#poly_r]    ; Put center of circle to (0,0) coordinate.

G1 F[150 * 60]
#i=0        ; Loop counter
WHILE [#i <= #faces] DO
      X[#poly_r * cos[#i * #angle + #aoff]] Y[#poly_r * sin[#i * #angle + #aoff]]
      #i++
END

; Now compare that to an inner 'regular' circle.
G1 X#cg2_r Y0
G2 X#cg2_r Y0 I[-#cg2_r] J0

G1 X#cg3_r Y0
G3 X#cg3_r Y0 I[-#cg3_r] J0
