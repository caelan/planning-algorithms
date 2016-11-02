;; Block A is on the table, block B on A and there is nothing on B.  A
;; water bucket, a brush, a A blue sprayer and a red paint can are on
;; the table and clear.  The goal is for A to be colored red and B
;; blue and the brush be clean. 

(define (problem 2)
  (:domain hw4)
  (:objects A B water-can brush sprayer paint-can)
  (:init (arm-empty)
	 (is-paint-can paint-can)
	 (is-brush brush)
	 (is-water-can water-can)
	 (is-sprayer sprayer)
	 (paints-color paint-can red)
	 (paints-color sprayer blue)
	 (clean brush)
	 (on B A)
	 (on-table A)
	 (on-table sprayer)
	 (on-table paint-can)
	 (on-table brush)
	 (on-table water-can)
	 (clear B)
	 (clear sprayer)
	 (clear paint-can)
	 (clear brush)
	 (clear water-can)
         ;;... block A is on the table ...
	 ;;... block B is on block A and there's nothing on B ...
         ;;... there is a blue sprayer on the table and nothing is on it ...
	 ;;... there is a red paint can on the table and noting is on it ...
	 ;;... there is a clean brush on the table and nothing is on it  ...
	 ;;... there is a water bucket on the table and nothing is on it ...
      )
  (:goal (and (arm-empty)
         (colored A red)
         (colored B blue)
	 (clean brush)
         ;;... A is red ...
         ;;... B is blue ...
         ;;... the brush is clean ...
         )))





