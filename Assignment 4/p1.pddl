;; There is only one block, A, which is on the table.  A can with
;; red paint is on the table.  There is a clean brush on the
;; table.  Our goal is to have A be red and the arm empty.

(define (problem 1)
  (:domain hw4)
  (:objects A paint-can brush)
  (:init (arm-empty)
	 (is-paint-can paint-can)
	 (is-brush brush)
	 (paints-color paint-can red)
	 (clean brush)
	 (on-table A)
	 (on-table paint-can)
	 (on-table brush)
	 (clear A)
	 (clear paint-can)
	 (clear brush)
         ;;... block A on the table with nothing on it ...
         ;;... a red paint can on the table with nothing on it ...
         ;;... a clean brush is on the table with nothing on it ...
	 )
  (:goal (and (arm-empty)
         (colored A red)
         ;;... A is red ...
         )))



