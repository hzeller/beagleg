#1=15       ; degrees
#2=-7.5     ; rotation offset. We want to start with a vertical line.
#3=50       ; polygon radius
#4=40       ; circle radius

G92 X[-#3] Y[-#3]    ; Put center of circle on convenient (0,0) coordinate.

G1 F[150 * 60]
; So, uhm, I think loops would be nice :)
X[#3 * cos[ 0 * #1 + #2]] Y[#3 * sin[ 0 * #1 + #2]]
X[#3 * cos[ 1 * #1 + #2]] Y[#3 * sin[ 1 * #1 + #2]]
X[#3 * cos[ 2 * #1 + #2]] Y[#3 * sin[ 2 * #1 + #2]]
X[#3 * cos[ 3 * #1 + #2]] Y[#3 * sin[ 3 * #1 + #2]]
X[#3 * cos[ 4 * #1 + #2]] Y[#3 * sin[ 4 * #1 + #2]]
X[#3 * cos[ 5 * #1 + #2]] Y[#3 * sin[ 5 * #1 + #2]]
X[#3 * cos[ 6 * #1 + #2]] Y[#3 * sin[ 6 * #1 + #2]]
X[#3 * cos[ 7 * #1 + #2]] Y[#3 * sin[ 7 * #1 + #2]]
X[#3 * cos[ 8 * #1 + #2]] Y[#3 * sin[ 8 * #1 + #2]]
X[#3 * cos[ 9 * #1 + #2]] Y[#3 * sin[ 9 * #1 + #2]]
X[#3 * cos[10 * #1 + #2]] Y[#3 * sin[10 * #1 + #2]]
X[#3 * cos[11 * #1 + #2]] Y[#3 * sin[11 * #1 + #2]]
X[#3 * cos[12 * #1 + #2]] Y[#3 * sin[12 * #1 + #2]]
X[#3 * cos[13 * #1 + #2]] Y[#3 * sin[13 * #1 + #2]]
X[#3 * cos[14 * #1 + #2]] Y[#3 * sin[14 * #1 + #2]]
X[#3 * cos[15 * #1 + #2]] Y[#3 * sin[15 * #1 + #2]]
X[#3 * cos[16 * #1 + #2]] Y[#3 * sin[16 * #1 + #2]]
X[#3 * cos[17 * #1 + #2]] Y[#3 * sin[17 * #1 + #2]]
X[#3 * cos[18 * #1 + #2]] Y[#3 * sin[18 * #1 + #2]]
X[#3 * cos[19 * #1 + #2]] Y[#3 * sin[19 * #1 + #2]]
X[#3 * cos[20 * #1 + #2]] Y[#3 * sin[20 * #1 + #2]]
X[#3 * cos[21 * #1 + #2]] Y[#3 * sin[21 * #1 + #2]]
X[#3 * cos[22 * #1 + #2]] Y[#3 * sin[22 * #1 + #2]]
X[#3 * cos[23 * #1 + #2]] Y[#3 * sin[23 * #1 + #2]]
X[#3 * cos[24 * #1 + #2]] Y[#3 * sin[24 * #1 + #2]]

; Now compare that to an inner 'regular' circle.
G1 X#4 Y0
G2 X#4 Y0 I[-#4] J0