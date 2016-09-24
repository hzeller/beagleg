G1 F[150 * 60]  (150mm/s)
; Chop each movement into smaller segments to see if the
; planner does enough look-ahead to start decelerating early
; enough.
#step_size=[100/20]

; right
#d=0
while [ #d <= 100 ] DO
  X#d
  #d+=#step_size
END

; up
#d=0
while [ #d <= 100 ] DO
  Y#d
  #d+=#step_size
END

; left
#d=100
while [ #d >= 0 ] DO
  X#d
  #d-=#step_size
END

; down
#d=100
while [ #d >= 0 ] DO
  Y#d
  #d-=#step_size
END

; Diagonal top right
#d=0
while [ #d <= 100 ] DO
  X#d Y#d
  #d+=#step_size
END
