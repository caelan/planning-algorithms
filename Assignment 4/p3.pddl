;; three blocks (A B and C) are on the table along with three sprayers
;; (red, green, blue), three paint cans (red, green, blue), a water
;; bucket and a clean brush.  Paint A, B and C red, blue and green,
;; respectively. End with the arm empty and the brush clean.

(define (problem 3)
  (:domain hw4)
  (:objects A B C sprayer-1 sprayer-2 sprayer-3 paint-can-1 paint-can-2 paint-can-3 water-can brush)
  (:init (arm-empty)
	 (is-sprayer sprayer-1)
	 (is-sprayer sprayer-2)
	 (is-sprayer sprayer-3)
	 (is-paint-can paint-can-1)
	 (is-paint-can paint-can-2)
	 (is-paint-can paint-can-3)
	 (is-brush brush)
	 (is-water-can water-can)

	 (paints-color sprayer-1 red)
	 (paints-color sprayer-2 blue)
	 (paints-color sprayer-3 green)
	 (paints-color paint-can-1 red)
	 (paints-color paint-can-2 blue)
	 (paints-color paint-can-3 green)

	 (clean brush)
	 (on-table A)
	 (on-table B)
	 (on-table C)
	 (on-table sprayer-1)
	 (on-table sprayer-2)
	 (on-table sprayer-3)
	 (on-table paint-can-1)
	 (on-table paint-can-2)
	 (on-table paint-can-3)
	 (on-table brush)
	 (on-table water-can)     

	 (clear A)
	 (clear B)
	 (clear C)
	 (clear sprayer-1)
	 (clear sprayer-2)
	 (clear sprayer-3)
	 (clear paint-can-1)
	 (clear paint-can-2)
	 (clear paint-can-3)
	 (clear brush)
	 (clear water-can)  
	 )
  (:goal (and (arm-empty)
         (colored A red)
         (colored B blue)
         (colored C green)
	 (clean brush)
         ;;... a is red, b is blue, C is green and the brush is clean...
	 )))
    




