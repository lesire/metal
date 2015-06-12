
(define (domain action)
  (:requirements :strips :typing :durative-actions :equality :agents-def)
  (:types robot - object
    a g v - robot
    loc - object
    loc-wp loc-obs - loc)
  (:predicates (explored ?z - loc-obs)
    (at-r ?r - robot ?z - loc-wp)
    (adjacent ?from ?to - loc-wp)
    (robot-allowed ?r - robot ?wp - loc-wp)
    (visible ?r - robot ?from - loc-wp ?to - loc-obs)
    (visible-com ?r1 ?r2 - robot ?wp1 ?wp2 - loc-wp)
    (in-communication-at ?r1 ?r2 - robot ?l1 ?l2 - loc-wp)
    (in-communication ?r1 ?r2 - robot)
    (have-been-in-com ?r1 ?r2 - robot))
  (:functions (distance ?from ?to - loc-wp))

  
(:durative-action move
  :parameters (?r - robot
       ?from ?to - loc-wp)
  :agent (?r)
  
  :duration (= ?duration (distance ?from ?to))
  :condition (and (over all (robot-allowed ?r ?from)) (over all (robot-allowed ?r ?to)) (over all (adjacent ?from ?to)) (over all (not (= ?from ?to))) (at start (at-r ?r ?from)))
  :effect (and (at end (at-r ?r ?to)) (at start (not (at-r ?r ?from))))
  
  
)


(:durative-action observe
  :parameters (?r - agv
       ?from - loc-wp
       ?to - loc-obs)
  :agent (?r)
  
  :duration (= ?duration 0.5)
  :condition (and (over all (robot-allowed ?r ?from)) (over all (visible ?r ?from ?to)) (over all (at-r ?r ?from)))
  :effect (and (at end (explored ?to)))
  
  
)


(:durative-action communicate
  :parameters (?r1 ?r2 - robot
       ?l1 ?l2 - loc-wp)
  :agent (?r1)
  
  :duration (= ?duration 1)
  :condition (and (over all (robot-allowed ?r1 ?l1)) (over all (robot-allowed ?r2 ?l2)) (over all (not (= ?r1 ?r2))) (over all (visible-com ?r1 ?r2 ?l1 ?l2)) (over all (at-r ?r1 ?l1)))
  :effect (and (at start (in-communication-at ?r1 ?r2 ?l1 ?l2)) (at end (not (in-communication-at ?r1 ?r2 ?l1 ?l2))))
  
  
)


(:durative-action communicate-meta
  :parameters (?r1 ?r2 - robot
       ?l1 ?l2 - loc-wp)
  :agent (?r1)
  
  :duration (= ?duration 0.5)
  :condition (and (over all (robot-allowed ?r1 ?l1)) (over all (robot-allowed ?r2 ?l2)) (over all (not (= ?r1 ?r2))) (over all (visible-com ?r1 ?r2 ?l1 ?l2)) (over all (in-communication-at ?r1 ?r2 ?l1 ?l2)) (over all (in-communication-at ?r2 ?r1 ?l2 ?l1)))
  :effect (and (at start (in-communication ?r1 ?r2)) (at start (in-communication ?r2 ?r1)) (at end (not (in-communication ?r1 ?r2))) (at end (not (in-communication ?r2 ?r1))))
  
  
)


(:durative-action has-communicated
  :parameters (?r1 ?r2 - robot)
  :agent (?r1)
  
  :duration (= ?duration 1)
  :condition (and (over all (not (= ?r1 ?r2))) (at start (in-communication ?r1 ?r2)) (at start (in-communication ?r2 ?r1)))
  :effect (and (at end (have-been-in-com ?r1 ?r2)) (at end (have-been-in-com ?r2 ?r1)))
  
  
)


(:durative-action init
  :parameters (?r - robot
       ?l - loc-wp)
  :agent (?r)
  
  :duration (= ?duration 1)
  :condition (and (over all (at-r ?r ?l)))
  :effect (and (at end (at-r ?r ?l)))
  
  
)

)
