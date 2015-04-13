
(define (domain action)
  (:requirements :strips :typing :durative-actions :equality)
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
    (in-communication ?r1 ?r2 - robot)
    (have-been-in-com ?r1 ?r2 - robot))
  (:functions (distance ?from ?to - loc-wp))

  
(:durative-action move
  :parameters (?r - robot
       ?from ?to - loc-wp)
  
  :duration (= ?duration (distance ?from ?to))
  :condition (and (over all (robot-allowed ?r ?from)) (over all (robot-allowed ?r ?to)) (over all (adjacent ?from ?to)) (over all (not (= ?from ?to))) (at start (at-r ?r ?from)))
  :effect (and (at end (at-r ?r ?to)) (at start (not (at-r ?r ?from))))
  
  
)


(:durative-action observe
  :parameters (?r - agv
       ?from - loc-wp
       ?to - loc-obs)
  
  :duration (= ?duration 0.1)
  :condition (and (over all (robot-allowed ?r ?from)) (over all (visible ?r ?from ?to)) (over all (at-r ?r ?from)))
  :effect (and (at end (explored ?to)))
  
  
)


(:durative-action communicate
  :parameters (?axv1 ?axv2 - robot
       ?laxv1 ?laxv2 - loc-wp)
  
  :duration (= ?duration 1)
  :condition (and (over all (robot-allowed ?axv1 ?laxv1)) (over all (robot-allowed ?axv2 ?laxv2)) (over all (not (= ?axv1 ?axv2))) (over all (visible-com ?axv1 ?axv2 ?laxv1 ?laxv2)) (over all (at-r ?axv1 ?laxv1)) (over all (at-r ?axv2 ?laxv2)))
  :effect (and (at start (in-communication ?axv1 ?axv2)) (at end (not (in-communication ?axv1 ?axv2))) (at start (in-communication ?axv2 ?axv1)) (at end (not (in-communication ?axv2 ?axv1))))
  
  
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


(:durative-action init
  :parameters (?r - robot
       ?l - loc-wp)
  
  :duration (= ?duration 1)
  :condition (and (over all (at-r ?r ?l)))
  :effect ()
  :side-effect (and (at end (at-r ?r ?l)))
  
)

)
