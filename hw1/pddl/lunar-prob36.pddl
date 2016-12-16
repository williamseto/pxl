

;; row first, col second for positions


(define (problem lunar2)
  (:domain lunar-lockout)
  (:objects blue purple red green orange yellow p1 p2 p3 p4 p5)
  (:init
   (ship purple) (ship red) (ship green) (ship orange) (ship yellow) (ship blue)
   (position p1) (position p2) (position p3) (position p4) (position p5)
   
   (inc p5 p4) (inc p5 p3) (inc p5 p2) (inc p5 p1)
   (inc p4 p3) (inc p4 p2) (inc p4 p1) 
   (inc p3 p2) (inc p3 p1) 
   (inc p2 p1)
   (dec p1 p2) (dec p1 p3) (dec p1 p4) (dec p1 p5)
   (dec p2 p3) (dec p2 p4) (dec p2 p5) 
   (dec p3 p4) (dec p3 p5) 
   (dec p4 p5)

   (gt p5 p4) (gt p4 p3) (gt p3 p2) (gt p2 p1)
   (lt p1 p2) (lt p2 p3) (lt p3 p4) (lt p4 p5)


   ;; here comes our specific problem
   (at orange p2 p1) (at green p2 p5) (at purple p4 p1) (at yellow p4 p5)
   (at blue p5 p3) (at red p1 p3)

  )
  (:goal
   (at red p3 p3))
)
