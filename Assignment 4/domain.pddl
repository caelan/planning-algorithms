;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; HW4 blocks world + painting
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain hw4)
  (:requirements :strips :typing)
  (:constants red green blue)

  (:predicates (on ?x ?y) ;; x on y
	       (on-table ?x)
	       (clear ?x)
	       (arm-empty)
	       (holding ?x)
	       (colored ?x ?c)
	       (paints-color ?x ?c)
	       (clean ?x)
	       (is-sprayer ?x)
	       (is-brush ?x)
	       (is-paint-can ?x)
	       (is-water-can ?x))
  (:action pick-up
	     :parameters (?ob1)
	     :precondition (and (clear ?ob1) (on-table ?ob1) (arm-empty))
	     :effect
	     (and (not (on-table ?ob1))
		   (not (clear ?ob1))
		   (not (arm-empty))
		   (holding ?ob1)))
  (:action put-down
	     :parameters (?ob)
	     :precondition (holding ?ob)
	     :effect
	     (and (not (holding ?ob))
		   (clear ?ob)
		   (arm-empty)
		   (on-table ?ob)))
  (:action stack
	     :parameters (?sob ?sunderob)
	     :precondition (and (holding ?sob) (clear ?sunderob))
	     :effect
	     (and (not (holding ?sob))
		   (not (clear ?sunderob))
		   (clear ?sob)
		   (arm-empty)
		   (on ?sob ?sunderob)))
  (:action unstack
	     :parameters (?sob ?sunderob)
	     :precondition (and (on ?sob ?sunderob) (clear ?sob) (arm-empty))
	     :effect
	     (and (holding ?sob)
		   (clear ?sunderob)
		   (not (clear ?sob))
		   (not (arm-empty))
		   (not (on ?sob ?sunderob))))
  (:action spray-paint
	     :parameters (?ob ?sprayer ?color)
	     :precondition (and (on-table ?ob) (clear ?ob) (is-sprayer ?sprayer) (paints-color ?sprayer ?color) (holding ?sprayer))
	     :effect (colored ?ob ?color))
  (:action brush-paint
	     :parameters (?ob ?brush ?color)
	     :precondition (and (on-table ?ob) (clear ?ob) (is-brush ?brush) (paints-color ?brush ?color) (not (clean ?brush)) (holding ?brush))
	     :effect (colored ?ob ?color))
  (:action load-brush
	     :parameters (?brush ?paint-can ?color)
	     :precondition (and (is-brush ?brush) (clean ?brush) (is-paint-can ?paint-can) (clear ?paint-can) (paints-color ?paint-can ?color) (holding ?brush))
	     :effect (paints-color ?brush ?color) (not (clean ?brush)))
  (:action wash-brush
	     :parameters (?brush ?water-can ?color)
	     :precondition (and(is-brush ?brush) (is-water-can ?water-can) (clear ?water-can) (holding ?brush) (paints-color ?brush ?color))
	     :effect (clean ?brush) (not (paints-color ?brush ?color)))
)
