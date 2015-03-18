
(define (domain action-V)
  (:requirements :strips :typing :durative-actions :equality)
  (:types robot - object
    agv aav - robot
    aav1 aav2 - aav
    loc - object)
  (:predicates (explored ?z - loc)
    (at-r ?r - robot ?z - loc)
    (adjacent-aav ?from ?to - loc)
    (visible-aav ?from ?to - loc)
    (aav-point-allowed ?r - aav ?pt - loc)
    (adjacent-agv ?from ?to - loc)
    (visible-agv ?from ?to - loc)
    (visible-com ?from ?to - loc)
    (in-communication ?r1 ?r2 - robot)
    (have-been-in-com ?r1 ?r2 - robot))
  (:functions (distance-agv ?from ?to - loc)
    (distance-aav ?from ?to - loc))

  
(:durative-action move-agv
  :parameters (?r - agv
       ?from ?to - loc)
  
  :duration (= ?duration (distance-agv ?from ?to))
  :condition (and (at start (at-r ?r ?from)) (over all (adjacent-agv ?from ?to)) (over all (not (= ?from ?to))))
  :effect (and (at end (at-r ?r ?to)) (at start (not (at-r ?r ?from))))
  
  
)


(:durative-action move-aav
  :parameters (?r - aav
       ?from ?to - loc)
  
  :duration (= ?duration (distance-aav ?from ?to))
  :condition (and (at start (at-r ?r ?from)) (over all (adjacent-aav ?from ?to)) (over all (aav-point-allowed ?r ?to)) (over all (not (= ?from ?to))))
  :effect (and (at end (at-r ?r ?to)) (at start (not (at-r ?r ?from))))
  
  
)


(:durative-action observe-agv
  :parameters (?r - agv
       ?from ?to - loc)
  
  :duration (= ?duration 1)
  :condition (and (over all (at-r ?r ?from)) (over all (visible-agv ?from ?to)))
  :effect (and (at end (explored ?to)))
  
  
)


(:durative-action observe-aav
  :parameters (?r - aav
       ?from ?to - loc)
  
  :duration (= ?duration 1)
  :condition (and (over all (at-r ?r ?from)) (over all (visible-aav ?from ?to)))
  :effect (and (at end (explored ?to)))
  
  
)


(:durative-action communicate-aav-agv
  :parameters (?aav - aav
       ?agv - agv
       ?laav ?lagv - loc)
  
  :duration (= ?duration 1)
  :condition (and (over all (at-r ?aav ?laav)) (over all (at-r ?agv ?lagv)) (over all (visible-com ?laav ?lagv)))
  :effect (and (at start (in-communication ?aav ?agv)) (at end (not (in-communication ?aav ?agv))) (at start (in-communication ?agv ?aav)) (at end (not (in-communication ?agv ?aav))))
  
  
)


(:durative-action communicate-aav-aav
  :parameters (?aav1 ?aav2 - aav
       ?laav1 ?laav2 - loc)
  
  :duration (= ?duration 1)
  :condition (and (over all (at-r ?aav1 ?laav1)) (over all (at-r ?aav2 ?laav2)) (over all (visible-com ?laav1 ?laav2)) (over all (aav-point-allowed ?aav1 ?laav1)) (over all (aav-point-allowed ?aav2 ?laav2)))
  :effect (and (at start (in-communication ?aav1 ?aav2)) (at end (not (in-communication ?aav1 ?aav2))) (at start (in-communication ?aav2 ?aav1)) (at end (not (in-communication ?aav2 ?aav1))))
  
  
)


(:durative-action has-communicated
  :parameters (?r1 ?r2 - robot)
  
  :duration (= ?duration 1)
  :condition (and (at start (in-communication ?r1 ?r2)) (over all (not (= ?r1 ?r2))))
  :effect (and (at end (have-been-in-com ?r1 ?r2)))
  
  
)


(:durative-action has-communicated-goal
  :parameters (?r1 ?r2 - robot)
  
  :duration (= ?duration 1)
  :condition (and (at start (have-been-in-com ?r1 ?r2)) (over all (not (= ?r1 ?r2))))
  :effect (and (at end (not (have-been-in-com ?r1 ?r2))))
  
  
)

)
