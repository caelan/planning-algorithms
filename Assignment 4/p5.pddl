;; Block A is on the table, B is on A and C on B.  A red sprayer is on
;; the table.  A green paint can is on the sprayer and a blue paint
;; can on the green paint can and a water bucket on the blue can.  A
;; clean brush is on the water bucket.  The goal is to make A red, B
;; green and C blue and to have A on B, B on C and C on the table. The
;; brush should be clean and on top of the paint can with green paint.

(define (problem 5)
  (:domain hw4)
  (:objects A B C red-sprayer blue-paint-can green-paint-can water-can brush)
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
	 (on green-paint-can red-sprayer)
	 (on blue-paint-can green-paint-can)
	 (on water-can blue-paint-can)
	 (on brush water-can)     

	 (clear C)
	 (clear brush)
	 )
  (:goal (and (arm-empty) 
         (colored A red)
         (colored B green)
         (colored C blue)
	 (on A B)
	 (on B C)
	 (on-table C)
	 (on brush green-paint-can)
	 (clean brush)
	 )))
