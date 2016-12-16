


(define (domain lunar-lockout)
  (:requirements :strips :typing :equality)
  (:predicates
   (ship ?x) (position ?x)
   (at ?s ?x ?y) (blank ?x ?y)
   (inc ?p ?pp) (dec ?p ?pp)
   (gt ?p ?pp) (lt ?p ?pp)
  )

  (:action move-up
    :parameters (?t ?pr ?pc ?mr ?tr)
    :precondition (and
		   (ship ?t) (position ?pr) (position ?pc) (position ?mr) (position ?tr)
		   (dec ?mr ?pr)
       ;;(blank ?mr ?pc)
       (forall (?zz) (not (at ?zz ?mr ?pc)))
       (at ?t ?pr ?pc)

       (lt ?tr ?mr)
       ;;(not (blank ?tr ?pc))
       (exists (?z) (at ?z ?tr ?pc))
    )
    :effect (and (not (at ?t ?pr ?pc)) (at ?t ?mr ?pc)))

  (:action move-down
    :parameters (?t ?pr ?pc ?mr ?tr)
    :precondition (and
       (ship ?t) (position ?pr) (position ?pc) (position ?mr) (position ?tr)
       (inc ?mr ?pr) 
       (forall (?zz) (not (at ?zz ?mr ?pc)))

       (forall (?tt) 
        (or (ship ?tt) (dec ?tt ?pr) (= ?tt ?tr) (= ?tt ?pr) (and (inc ?tt ?pr) (not (exists (?ttt) (at ?ttt ?tt ?pc)) ))) 
       )
       (at ?t ?pr ?pc)

       (gt ?tr ?mr) (exists (?z) (at ?z ?tr ?pc))
    )
    :effect (and (not (at ?t ?pr ?pc)) (at ?t ?mr ?pc)))

  (:action move-left
    :parameters (?t ?pr ?pc ?mc ?tc)
    :precondition (and
		   (ship ?t) (position ?pr) (position ?pc) (position ?mc) (position ?tc)
		   (dec ?mc ?pc) 
       (forall (?zz) (not (at ?zz ?pr ?mc))) (at ?t ?pr ?pc)

       (lt ?tc ?mc) (exists (?z) (at ?z ?pr ?tc))
    )
    :effect (and (not (at ?t ?pr ?pc)) (at ?t ?pr ?mc)))

  (:action move-right
    :parameters (?t ?pr ?pc ?mc ?tc)
    :precondition (and
       (ship ?t) (position ?pr) (position ?pc) (position ?mc) (position ?tc)
       (inc ?mc ?pc) 
       (forall (?zz) (not (at ?zz ?pr ?mc))) (at ?t ?pr ?pc)

       (gt ?tc ?mc) (exists (?z) (at ?z ?pr ?tc))
    )
    :effect (and (not (at ?t ?pr ?pc)) (at ?t ?pr ?mc)))
  )
