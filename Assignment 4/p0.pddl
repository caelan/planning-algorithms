;; There is only one block, A, which is on the table.  A sprayer with
;; red paint is on the table.  Our goal is to have A be red and the
;; arm empty.

(define (problem 0)
  (:domain hw4)
  (:objects A sprayer)
  (:init (arm-empty)
	 (is-sprayer sprayer)
	 (paints-color sprayer red)
	 (on-table A)
	 (on-table sprayer)
	 (clear A)
	 (clear sprayer)
         ;;... there is a block A on the table with nothing on it...
         ;;... there is a red sprayer on the table with nothing on it...
         )
  (:goal (and (arm-empty)
         (colored A red)
         ;;...A is red...
         )))



