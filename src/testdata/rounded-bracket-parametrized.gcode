; cut a L shape with rounded corners
#1=[150 * 60]   (feedrate)
#2=100          (side length)
#3=20           (inner space)
#4=5            (corner radius)

g28
g1 f#1
g1 x[#2-#4]
g3 x#2 y#4 j#4
g1 y[#3-#4]
g3 x[#2-#4] y#3 i [-abs[#4]]
g1 x[#3+#4]
g2 x#3 y[#3+#4] j#4
g1 y[#2-#4]
g3 x[#3-#4] y#2 i [-abs[#4]]
g1 x#4
g3 x0 y[#2-#4] j [-abs[#4]]
g1 x0 y0

; slightly smaller L using radius arcs
#5=4            (offset)
#2=[#2-[#5*2]]
#3=[#3-#5]
#4=[#4-[#5/2]]
g1 x#5 y#5
g1 x[#5+#2-#4]
g3 x[#5+#2] y[#5+#4] r#4
g1 y[#3-#4]
g3 x[#5+#2-#4] y[#3] r#4
g1 x[#3+#4]
g2 x#3 y[#3+#4] r#4
;g2 x#3 y[#3+#4] r [-abs[#4]]      (negative radius takes a longer path)
g1 y[#5+#2-#4]
g3 x[#3-#4] y[#5+#2] r#4
g1 x[#5+#4]
g3 x#5 y[#5+#2-#4] r#4
g1 x#5 y#5

g1 x0 y0
m18             (turn off motors)
m107            (turn off fan)
