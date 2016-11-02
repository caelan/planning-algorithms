;; Block A is on the table, B is on A and C on B.  On the table are a
;; water bucket, a red sprayer, cans of blue and green paint and a
;; clean brush.  The goal is to make A red, B green and C blue and to
;; have A on B, B on C and C on the table and the brush clean and arm
;; empty.

(define (problem 4)
  (:domain hw4)
  (:objects A B C water-can red-sprayer blue-paint-can green-paint-can brush)
  (:init (arm-empty)
	 (is-sprayer red-sprayer)
	 (is-paint-can blue-paint-can)
	 (is-paint-can green-paint-can)
	 (is-brush brush)
	 (is-water-can water-can)

	 (paints-color red-sprayer red)
	 (paints-color blue-paint-can blue)
	 (paints-color green-paint-can green)

	 (clean brush)
	 (on-table A)
	 (on B A)
	 (on C B)
	 (on-table red-sprayer)
	 (on-table blue-paint-can)
	 (on-table green-paint-can)
	 (on-table brush)
	 (on-table water-can)     

	 (clear C)
	 (clear red-sprayer)
	 (clear blue-paint-can)
	 (clear green-paint-can)
	 (clear brush)
	 (clear water-can)  
	 )
  (:goal (and (arm-empty) 
         (colored A red)
         (colored B green)
         (colored C blue)
	 (on A B)
	 (on B C)
	 (on-table C)
	 (clean brush)
	 )))


