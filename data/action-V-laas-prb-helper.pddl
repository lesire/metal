
(define (domain-helper action)
    (:options :abstractOnly :erasePlansWhenAbstractMet)
    (:allowed-actions move communicate has-communicated)
    (:low-priority-predicates )

    
(:action patrol_mana__agv_-2186_-4073_0__agv_1763_-8654_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agv_-2186_-4073_0))
  :effect (and (explored pt_obs_-800_-6200) (explored pt_obs_613_-8154) (explored pt_obs_2054_-7404) (at-r mana agv_1763_-8654_0))
  
  :methods(

      :method patrol_mana__agv_-2186_-4073_0__agv_1763_-8654_0_d_1
      :actions (move-0 (move mana agv_-2186_-4073_0 agv_-806_-6304_0))
        (move-1 (move mana agv_-806_-6304_0 agv_1763_-8654_0))
        (explore-0 (observe mana agv_-806_-6304_0 pt_obs_-800_-6200))
        (explore-1 (observe mana agv_1763_-8654_0 pt_obs_613_-8154))
        (explore-2 (observe mana agv_1763_-8654_0 pt_obs_2054_-7404))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agv_-2186_-4073_0))
        (move-0 move-1 (at-r mana agv_-806_-6304_0))
        (move-1 :goal (at-r mana agv_1763_-8654_0))
        (explore-0 :goal (explored pt_obs_-800_-6200))
        (move-0 explore-0 (at-r mana agv_-806_-6304_0))
        (explore-1 :goal (explored pt_obs_613_-8154))
        (move-1 explore-1 (at-r mana agv_1763_-8654_0))
        (explore-2 :goal (explored pt_obs_2054_-7404))
        (move-1 explore-2 (at-r mana agv_1763_-8654_0))
      :temporal-links (explore-0 move-1)

  )
)


(:action patrol_mana__agv_1763_-8654_0__agv_-2186_-4073_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agv_1763_-8654_0))
  :effect (and (explored pt_obs_-800_-6200) (explored pt_obs_613_-8154) (explored pt_obs_2054_-7404) (at-r mana agv_-2186_-4073_0))
  
  :methods(

      :method patrol_mana__agv_1763_-8654_0__agv_-2186_-4073_0_i_1
      :actions (move-0 (move mana agv_1763_-8654_0 agv_-806_-6304_0))
        (move-1 (move mana agv_-806_-6304_0 agv_-2186_-4073_0))
        (explore-0 (observe mana agv_1763_-8654_0 pt_obs_613_-8154))
        (explore-1 (observe mana agv_1763_-8654_0 pt_obs_2054_-7404))
        (explore-2 (observe mana agv_-806_-6304_0 pt_obs_-800_-6200))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agv_1763_-8654_0))
        (move-0 move-1 (at-r mana agv_-806_-6304_0))
        (move-1 :goal (at-r mana agv_-2186_-4073_0))
        (explore-0 :goal (explored pt_obs_613_-8154))
        (:init explore-0 (at-r mana agv_1763_-8654_0))
        (explore-1 :goal (explored pt_obs_2054_-7404))
        (:init explore-1 (at-r mana agv_1763_-8654_0))
        (explore-2 :goal (explored pt_obs_-800_-6200))
        (move-0 explore-2 (at-r mana agv_-806_-6304_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-0)
        (explore-2 move-1)

  )
)


(:action patrol_mana__agv_1763_-8654_0__agv_3713_-6384_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agv_1763_-8654_0))
  :effect (and (explored pt_obs_613_-8154) (explored pt_obs_2054_-7404) (at-r mana agv_3713_-6384_0))
  
  :methods(

      :method patrol_mana__agv_1763_-8654_0__agv_3713_-6384_0_d_0
      :actions (move-0 (move mana agv_1763_-8654_0 agv_2733_-7504_0))
        (move-1 (move mana agv_2733_-7504_0 agv_3713_-6384_0))
        (explore-0 (observe mana agv_1763_-8654_0 pt_obs_613_-8154))
        (explore-1 (observe mana agv_1763_-8654_0 pt_obs_2054_-7404))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agv_1763_-8654_0))
        (move-0 move-1 (at-r mana agv_2733_-7504_0))
        (move-1 :goal (at-r mana agv_3713_-6384_0))
        (explore-0 :goal (explored pt_obs_613_-8154))
        (:init explore-0 (at-r mana agv_1763_-8654_0))
        (explore-1 :goal (explored pt_obs_2054_-7404))
        (:init explore-1 (at-r mana agv_1763_-8654_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-0)

  )
)


(:action patrol_mana__agv_3713_-6384_0__agv_1763_-8654_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agv_3713_-6384_0))
  :effect (and (explored pt_obs_613_-8154) (explored pt_obs_2054_-7404) (at-r mana agv_1763_-8654_0))
  
  :methods(

      :method patrol_mana__agv_3713_-6384_0__agv_1763_-8654_0_i_0
      :actions (move-0 (move mana agv_3713_-6384_0 agv_2733_-7504_0))
        (move-1 (move mana agv_2733_-7504_0 agv_1763_-8654_0))
        (explore-0 (observe mana agv_1763_-8654_0 pt_obs_613_-8154))
        (explore-1 (observe mana agv_1763_-8654_0 pt_obs_2054_-7404))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agv_3713_-6384_0))
        (move-0 move-1 (at-r mana agv_2733_-7504_0))
        (move-1 :goal (at-r mana agv_1763_-8654_0))
        (explore-0 :goal (explored pt_obs_613_-8154))
        (move-1 explore-0 (at-r mana agv_1763_-8654_0))
        (explore-1 :goal (explored pt_obs_2054_-7404))
        (move-1 explore-1 (at-r mana agv_1763_-8654_0))
      :temporal-links 

  )
)


(:action patrol_mana__agv_533_-1973_0__agv_-2186_-4073_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agv_533_-1973_0))
  :effect (and (explored pt_obs_-1486_-3573) (at-r mana agv_-2186_-4073_0))
  
  :methods(

      :method patrol_mana__agv_533_-1973_0__agv_-2186_-4073_0_d_3
      :actions (move-0 (move mana agv_533_-1973_0 agv_-876_-3163_0))
        (move-1 (move mana agv_-876_-3163_0 agv_-2186_-4073_0))
        (explore-0 (observe mana agv_-876_-3163_0 pt_obs_-1486_-3573))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agv_533_-1973_0))
        (move-0 move-1 (at-r mana agv_-876_-3163_0))
        (move-1 :goal (at-r mana agv_-2186_-4073_0))
        (explore-0 :goal (explored pt_obs_-1486_-3573))
        (move-0 explore-0 (at-r mana agv_-876_-3163_0))
      :temporal-links (explore-0 move-1)

  )
)


(:action patrol_mana__agv_-2186_-4073_0__agv_533_-1973_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agv_-2186_-4073_0))
  :effect (and (explored pt_obs_-1486_-3573) (at-r mana agv_533_-1973_0))
  
  :methods(

      :method patrol_mana__agv_-2186_-4073_0__agv_533_-1973_0_i_3
      :actions (move-0 (move mana agv_-2186_-4073_0 agv_-876_-3163_0))
        (move-1 (move mana agv_-876_-3163_0 agv_533_-1973_0))
        (explore-0 (observe mana agv_-876_-3163_0 pt_obs_-1486_-3573))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agv_-2186_-4073_0))
        (move-0 move-1 (at-r mana agv_-876_-3163_0))
        (move-1 :goal (at-r mana agv_533_-1973_0))
        (explore-0 :goal (explored pt_obs_-1486_-3573))
        (move-0 explore-0 (at-r mana agv_-876_-3163_0))
      :temporal-links (explore-0 move-1)

  )
)


(:action patrol_mana__agv_-876_-3163_0__agv_2733_-7504_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agv_-876_-3163_0))
  :effect (and (explored pt_obs_-1486_-3573) (explored pt_obs_2054_-7404) (at-r mana agv_2733_-7504_0))
  
  :methods(

      :method patrol_mana__agv_-876_-3163_0__agv_2733_-7504_0_d_2
      :actions (move-0 (move mana agv_-876_-3163_0 agv_713_-5404_0))
        (move-1 (move mana agv_713_-5404_0 agv_2733_-7504_0))
        (explore-0 (observe mana agv_-876_-3163_0 pt_obs_-1486_-3573))
        (explore-1 (observe mana agv_2733_-7504_0 pt_obs_2054_-7404))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agv_-876_-3163_0))
        (move-0 move-1 (at-r mana agv_713_-5404_0))
        (move-1 :goal (at-r mana agv_2733_-7504_0))
        (explore-0 :goal (explored pt_obs_-1486_-3573))
        (:init explore-0 (at-r mana agv_-876_-3163_0))
        (explore-1 :goal (explored pt_obs_2054_-7404))
        (move-1 explore-1 (at-r mana agv_2733_-7504_0))
      :temporal-links (explore-0 move-0)

  )
)


(:action patrol_mana__agv_2733_-7504_0__agv_-876_-3163_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agv_2733_-7504_0))
  :effect (and (explored pt_obs_-1486_-3573) (explored pt_obs_2054_-7404) (at-r mana agv_-876_-3163_0))
  
  :methods(

      :method patrol_mana__agv_2733_-7504_0__agv_-876_-3163_0_i_2
      :actions (move-0 (move mana agv_2733_-7504_0 agv_713_-5404_0))
        (move-1 (move mana agv_713_-5404_0 agv_-876_-3163_0))
        (explore-0 (observe mana agv_2733_-7504_0 pt_obs_2054_-7404))
        (explore-1 (observe mana agv_-876_-3163_0 pt_obs_-1486_-3573))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agv_2733_-7504_0))
        (move-0 move-1 (at-r mana agv_713_-5404_0))
        (move-1 :goal (at-r mana agv_-876_-3163_0))
        (explore-0 :goal (explored pt_obs_2054_-7404))
        (:init explore-0 (at-r mana agv_2733_-7504_0))
        (explore-1 :goal (explored pt_obs_-1486_-3573))
        (move-1 explore-1 (at-r mana agv_-876_-3163_0))
      :temporal-links (explore-0 move-0)

  )
)


(:action patrol_mana__agv_3713_-6384_0__agv_4623_-863_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agv_3713_-6384_0))
  :effect (and (explored pt_obs_4154_-4043) (explored pt_obs_4563_-1193) (at-r mana agv_4623_-863_0))
  
  :methods(

      :method patrol_mana__agv_3713_-6384_0__agv_4623_-863_0_d_5
      :actions (move-0 (move mana agv_3713_-6384_0 agv_4183_-3793_0))
        (move-1 (move mana agv_4183_-3793_0 agv_4623_-863_0))
        (explore-0 (observe mana agv_4183_-3793_0 pt_obs_4154_-4043))
        (explore-1 (observe mana agv_4623_-863_0 pt_obs_4563_-1193))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agv_3713_-6384_0))
        (move-0 move-1 (at-r mana agv_4183_-3793_0))
        (move-1 :goal (at-r mana agv_4623_-863_0))
        (explore-0 :goal (explored pt_obs_4154_-4043))
        (move-0 explore-0 (at-r mana agv_4183_-3793_0))
        (explore-1 :goal (explored pt_obs_4563_-1193))
        (move-1 explore-1 (at-r mana agv_4623_-863_0))
      :temporal-links (explore-0 move-1)

  )
)


(:action patrol_mana__agv_4623_-863_0__agv_3713_-6384_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agv_4623_-863_0))
  :effect (and (explored pt_obs_4154_-4043) (explored pt_obs_4563_-1193) (at-r mana agv_3713_-6384_0))
  
  :methods(

      :method patrol_mana__agv_4623_-863_0__agv_3713_-6384_0_i_5
      :actions (move-0 (move mana agv_4623_-863_0 agv_4183_-3793_0))
        (move-1 (move mana agv_4183_-3793_0 agv_3713_-6384_0))
        (explore-0 (observe mana agv_4623_-863_0 pt_obs_4563_-1193))
        (explore-1 (observe mana agv_4183_-3793_0 pt_obs_4154_-4043))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agv_4623_-863_0))
        (move-0 move-1 (at-r mana agv_4183_-3793_0))
        (move-1 :goal (at-r mana agv_3713_-6384_0))
        (explore-0 :goal (explored pt_obs_4563_-1193))
        (:init explore-0 (at-r mana agv_4623_-863_0))
        (explore-1 :goal (explored pt_obs_4154_-4043))
        (move-0 explore-1 (at-r mana agv_4183_-3793_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)

  )
)


(:action patrol_mana__agv_-26_-163_0__agv_4623_-863_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agv_-26_-163_0))
  :effect (and (explored pt_obs_-606_-223) (explored pt_obs_4563_-1193) (at-r mana agv_4623_-863_0))
  
  :methods(

      :method patrol_mana__agv_-26_-163_0__agv_4623_-863_0_d_4
      :actions (move-0 (move mana agv_-26_-163_0 agv_1533_445_0))
        (move-1 (move mana agv_1533_445_0 agv_3043_-173_0))
        (move-2 (move mana agv_3043_-173_0 agv_4004_286_0))
        (move-3 (move mana agv_4004_286_0 agv_4623_-863_0))
        (explore-0 (observe mana agv_-26_-163_0 pt_obs_-606_-223))
        (explore-1 (observe mana agv_4623_-863_0 pt_obs_4563_-1193))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agv_-26_-163_0))
        (move-0 move-1 (at-r mana agv_1533_445_0))
        (move-1 move-2 (at-r mana agv_3043_-173_0))
        (move-2 move-3 (at-r mana agv_4004_286_0))
        (move-3 :goal (at-r mana agv_4623_-863_0))
        (explore-0 :goal (explored pt_obs_-606_-223))
        (:init explore-0 (at-r mana agv_-26_-163_0))
        (explore-1 :goal (explored pt_obs_4563_-1193))
        (move-3 explore-1 (at-r mana agv_4623_-863_0))
      :temporal-links (explore-0 move-0)

  )
)


(:action patrol_mana__agv_4623_-863_0__agv_-26_-163_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agv_4623_-863_0))
  :effect (and (explored pt_obs_-606_-223) (explored pt_obs_4563_-1193) (at-r mana agv_-26_-163_0))
  
  :methods(

      :method patrol_mana__agv_4623_-863_0__agv_-26_-163_0_i_4
      :actions (move-0 (move mana agv_4623_-863_0 agv_4004_286_0))
        (move-1 (move mana agv_4004_286_0 agv_3043_-173_0))
        (move-2 (move mana agv_3043_-173_0 agv_1533_445_0))
        (move-3 (move mana agv_1533_445_0 agv_-26_-163_0))
        (explore-0 (observe mana agv_4623_-863_0 pt_obs_4563_-1193))
        (explore-1 (observe mana agv_-26_-163_0 pt_obs_-606_-223))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agv_4623_-863_0))
        (move-0 move-1 (at-r mana agv_4004_286_0))
        (move-1 move-2 (at-r mana agv_3043_-173_0))
        (move-2 move-3 (at-r mana agv_1533_445_0))
        (move-3 :goal (at-r mana agv_-26_-163_0))
        (explore-0 :goal (explored pt_obs_4563_-1193))
        (:init explore-0 (at-r mana agv_4623_-863_0))
        (explore-1 :goal (explored pt_obs_-606_-223))
        (move-3 explore-1 (at-r mana agv_-26_-163_0))
      :temporal-links (explore-0 move-0)

  )
)


(:action patrol_mana__agv_1323_-3473_0__agv_4623_-863_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agv_1323_-3473_0))
  :effect (and (explored pt_obs_1593_-3273) (explored pt_obs_4563_-1193) (at-r mana agv_4623_-863_0))
  
  :methods(

      :method patrol_mana__agv_1323_-3473_0__agv_4623_-863_0_d_7
      :actions (move-0 (move mana agv_1323_-3473_0 agv_4623_-863_0))
        (explore-0 (observe mana agv_1323_-3473_0 pt_obs_1593_-3273))
        (explore-1 (observe mana agv_4623_-863_0 pt_obs_4563_-1193))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agv_1323_-3473_0))
        (move-0 :goal (at-r mana agv_4623_-863_0))
        (explore-0 :goal (explored pt_obs_1593_-3273))
        (:init explore-0 (at-r mana agv_1323_-3473_0))
        (explore-1 :goal (explored pt_obs_4563_-1193))
        (move-0 explore-1 (at-r mana agv_4623_-863_0))
      :temporal-links (explore-0 move-0)

  )
)


(:action patrol_mana__agv_4623_-863_0__agv_1323_-3473_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agv_4623_-863_0))
  :effect (and (explored pt_obs_1593_-3273) (explored pt_obs_4563_-1193) (at-r mana agv_1323_-3473_0))
  
  :methods(

      :method patrol_mana__agv_4623_-863_0__agv_1323_-3473_0_i_7
      :actions (move-0 (move mana agv_4623_-863_0 agv_1323_-3473_0))
        (explore-0 (observe mana agv_4623_-863_0 pt_obs_4563_-1193))
        (explore-1 (observe mana agv_1323_-3473_0 pt_obs_1593_-3273))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agv_4623_-863_0))
        (move-0 :goal (at-r mana agv_1323_-3473_0))
        (explore-0 :goal (explored pt_obs_4563_-1193))
        (:init explore-0 (at-r mana agv_4623_-863_0))
        (explore-1 :goal (explored pt_obs_1593_-3273))
        (move-0 explore-1 (at-r mana agv_1323_-3473_0))
      :temporal-links (explore-0 move-0)

  )
)


(:action patrol_mana__agv_533_-1973_0__agv_3713_-6384_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agv_533_-1973_0))
  :effect (and (explored pt_obs_1593_-3273) (at-r mana agv_3713_-6384_0))
  
  :methods(

      :method patrol_mana__agv_533_-1973_0__agv_3713_-6384_0_d_6
      :actions (move-0 (move mana agv_533_-1973_0 agv_1323_-3473_0))
        (move-1 (move mana agv_1323_-3473_0 agv_3713_-6384_0))
        (explore-0 (observe mana agv_1323_-3473_0 pt_obs_1593_-3273))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agv_533_-1973_0))
        (move-0 move-1 (at-r mana agv_1323_-3473_0))
        (move-1 :goal (at-r mana agv_3713_-6384_0))
        (explore-0 :goal (explored pt_obs_1593_-3273))
        (move-0 explore-0 (at-r mana agv_1323_-3473_0))
      :temporal-links (explore-0 move-1)

  )
)


(:action patrol_mana__agv_3713_-6384_0__agv_533_-1973_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agv_3713_-6384_0))
  :effect (and (explored pt_obs_1593_-3273) (at-r mana agv_533_-1973_0))
  
  :methods(

      :method patrol_mana__agv_3713_-6384_0__agv_533_-1973_0_i_6
      :actions (move-0 (move mana agv_3713_-6384_0 agv_1323_-3473_0))
        (move-1 (move mana agv_1323_-3473_0 agv_533_-1973_0))
        (explore-0 (observe mana agv_1323_-3473_0 pt_obs_1593_-3273))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agv_3713_-6384_0))
        (move-0 move-1 (at-r mana agv_1323_-3473_0))
        (move-1 :goal (at-r mana agv_533_-1973_0))
        (explore-0 :goal (explored pt_obs_1593_-3273))
        (move-0 explore-0 (at-r mana agv_1323_-3473_0))
      :temporal-links (explore-0 move-1)

  )
)


(:action patrol_ressac2__ressac2_5493_-2154_0__ressac2_1800_-3100_0
  :parameters ()
  :conflict-with (at-r ressac2 *)
  
  :precondition (and (at-r ressac2 ressac2_5493_-2154_0))
  :effect (and (explored pt_obs_1593_-3273) (at-r ressac2 ressac2_1800_-3100_0))
  
  :methods(

      :method patrol_ressac2__ressac2_5493_-2154_0__ressac2_1800_-3100_0_d_1
      :actions (move-0 (move ressac2 ressac2_5493_-2154_0 ressac2_6493_-3763_0))
        (move-1 (move ressac2 ressac2_6493_-3763_0 ressac2_5023_-4943_0))
        (move-2 (move ressac2 ressac2_5023_-4943_0 ressac2_1800_-3100_0))
        (explore-0 (observe ressac2 ressac2_1800_-3100_0 pt_obs_1593_-3273))
      
      :precondition
      :causal-links (:init move-0 (at-r ressac2 ressac2_5493_-2154_0))
        (move-0 move-1 (at-r ressac2 ressac2_6493_-3763_0))
        (move-1 move-2 (at-r ressac2 ressac2_5023_-4943_0))
        (move-2 :goal (at-r ressac2 ressac2_1800_-3100_0))
        (explore-0 :goal (explored pt_obs_1593_-3273))
        (move-2 explore-0 (at-r ressac2 ressac2_1800_-3100_0))
      :temporal-links 

  )
)


(:action patrol_ressac2__ressac2_1800_-3100_0__ressac2_5493_-2154_0
  :parameters ()
  :conflict-with (at-r ressac2 *)
  
  :precondition (and (at-r ressac2 ressac2_1800_-3100_0))
  :effect (and (explored pt_obs_1593_-3273) (at-r ressac2 ressac2_5493_-2154_0))
  
  :methods(

      :method patrol_ressac2__ressac2_1800_-3100_0__ressac2_5493_-2154_0_i_1
      :actions (move-0 (move ressac2 ressac2_1800_-3100_0 ressac2_5023_-4943_0))
        (move-1 (move ressac2 ressac2_5023_-4943_0 ressac2_6493_-3763_0))
        (move-2 (move ressac2 ressac2_6493_-3763_0 ressac2_5493_-2154_0))
        (explore-0 (observe ressac2 ressac2_1800_-3100_0 pt_obs_1593_-3273))
      
      :precondition
      :causal-links (:init move-0 (at-r ressac2 ressac2_1800_-3100_0))
        (move-0 move-1 (at-r ressac2 ressac2_5023_-4943_0))
        (move-1 move-2 (at-r ressac2 ressac2_6493_-3763_0))
        (move-2 :goal (at-r ressac2 ressac2_5493_-2154_0))
        (explore-0 :goal (explored pt_obs_1593_-3273))
        (:init explore-0 (at-r ressac2 ressac2_1800_-3100_0))
      :temporal-links (explore-0 move-0)

  )
)


(:action patrol_ressac1__ressac1_2323_1426_0__ressac1_5004_-993_0
  :parameters ()
  :conflict-with (at-r ressac1 *)
  
  :precondition (and (at-r ressac1 ressac1_2323_1426_0))
  :effect (and (explored pt_obs_4563_-1193) (at-r ressac1 ressac1_5004_-993_0))
  
  :methods(

      :method patrol_ressac1__ressac1_2323_1426_0__ressac1_5004_-993_0_d_1
      :actions (move-0 (move ressac1 ressac1_2323_1426_0 ressac1_3504_916_0))
        (move-1 (move ressac1 ressac1_3504_916_0 ressac1_5004_-993_0))
        (explore-0 (observe ressac1 ressac1_5004_-993_0 pt_obs_4563_-1193))
      
      :precondition
      :causal-links (:init move-0 (at-r ressac1 ressac1_2323_1426_0))
        (move-0 move-1 (at-r ressac1 ressac1_3504_916_0))
        (move-1 :goal (at-r ressac1 ressac1_5004_-993_0))
        (explore-0 :goal (explored pt_obs_4563_-1193))
        (move-1 explore-0 (at-r ressac1 ressac1_5004_-993_0))
      :temporal-links 

  )
)


(:action patrol_ressac1__ressac1_5004_-993_0__ressac1_2323_1426_0
  :parameters ()
  :conflict-with (at-r ressac1 *)
  
  :precondition (and (at-r ressac1 ressac1_5004_-993_0))
  :effect (and (explored pt_obs_4563_-1193) (at-r ressac1 ressac1_2323_1426_0))
  
  :methods(

      :method patrol_ressac1__ressac1_5004_-993_0__ressac1_2323_1426_0_i_1
      :actions (move-0 (move ressac1 ressac1_5004_-993_0 ressac1_3504_916_0))
        (move-1 (move ressac1 ressac1_3504_916_0 ressac1_2323_1426_0))
        (explore-0 (observe ressac1 ressac1_5004_-993_0 pt_obs_4563_-1193))
      
      :precondition
      :causal-links (:init move-0 (at-r ressac1 ressac1_5004_-993_0))
        (move-0 move-1 (at-r ressac1 ressac1_3504_916_0))
        (move-1 :goal (at-r ressac1 ressac1_2323_1426_0))
        (explore-0 :goal (explored pt_obs_4563_-1193))
        (:init explore-0 (at-r ressac1 ressac1_5004_-993_0))
      :temporal-links (explore-0 move-0)

  )
)


(:action patrol_minnie__agv_-2186_-4073_0__agv_1763_-8654_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agv_-2186_-4073_0))
  :effect (and (explored pt_obs_-800_-6200) (explored pt_obs_613_-8154) (explored pt_obs_2054_-7404) (at-r minnie agv_1763_-8654_0))
  
  :methods(

      :method patrol_minnie__agv_-2186_-4073_0__agv_1763_-8654_0_d_1
      :actions (move-0 (move minnie agv_-2186_-4073_0 agv_-806_-6304_0))
        (move-1 (move minnie agv_-806_-6304_0 agv_1763_-8654_0))
        (explore-0 (observe minnie agv_-806_-6304_0 pt_obs_-800_-6200))
        (explore-1 (observe minnie agv_1763_-8654_0 pt_obs_613_-8154))
        (explore-2 (observe minnie agv_1763_-8654_0 pt_obs_2054_-7404))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agv_-2186_-4073_0))
        (move-0 move-1 (at-r minnie agv_-806_-6304_0))
        (move-1 :goal (at-r minnie agv_1763_-8654_0))
        (explore-0 :goal (explored pt_obs_-800_-6200))
        (move-0 explore-0 (at-r minnie agv_-806_-6304_0))
        (explore-1 :goal (explored pt_obs_613_-8154))
        (move-1 explore-1 (at-r minnie agv_1763_-8654_0))
        (explore-2 :goal (explored pt_obs_2054_-7404))
        (move-1 explore-2 (at-r minnie agv_1763_-8654_0))
      :temporal-links (explore-0 move-1)

  )
)


(:action patrol_minnie__agv_1763_-8654_0__agv_-2186_-4073_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agv_1763_-8654_0))
  :effect (and (explored pt_obs_-800_-6200) (explored pt_obs_613_-8154) (explored pt_obs_2054_-7404) (at-r minnie agv_-2186_-4073_0))
  
  :methods(

      :method patrol_minnie__agv_1763_-8654_0__agv_-2186_-4073_0_i_1
      :actions (move-0 (move minnie agv_1763_-8654_0 agv_-806_-6304_0))
        (move-1 (move minnie agv_-806_-6304_0 agv_-2186_-4073_0))
        (explore-0 (observe minnie agv_1763_-8654_0 pt_obs_613_-8154))
        (explore-1 (observe minnie agv_1763_-8654_0 pt_obs_2054_-7404))
        (explore-2 (observe minnie agv_-806_-6304_0 pt_obs_-800_-6200))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agv_1763_-8654_0))
        (move-0 move-1 (at-r minnie agv_-806_-6304_0))
        (move-1 :goal (at-r minnie agv_-2186_-4073_0))
        (explore-0 :goal (explored pt_obs_613_-8154))
        (:init explore-0 (at-r minnie agv_1763_-8654_0))
        (explore-1 :goal (explored pt_obs_2054_-7404))
        (:init explore-1 (at-r minnie agv_1763_-8654_0))
        (explore-2 :goal (explored pt_obs_-800_-6200))
        (move-0 explore-2 (at-r minnie agv_-806_-6304_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-0)
        (explore-2 move-1)

  )
)


(:action patrol_minnie__agv_1763_-8654_0__agv_3713_-6384_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agv_1763_-8654_0))
  :effect (and (explored pt_obs_613_-8154) (explored pt_obs_2054_-7404) (at-r minnie agv_3713_-6384_0))
  
  :methods(

      :method patrol_minnie__agv_1763_-8654_0__agv_3713_-6384_0_d_0
      :actions (move-0 (move minnie agv_1763_-8654_0 agv_2733_-7504_0))
        (move-1 (move minnie agv_2733_-7504_0 agv_3713_-6384_0))
        (explore-0 (observe minnie agv_1763_-8654_0 pt_obs_613_-8154))
        (explore-1 (observe minnie agv_1763_-8654_0 pt_obs_2054_-7404))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agv_1763_-8654_0))
        (move-0 move-1 (at-r minnie agv_2733_-7504_0))
        (move-1 :goal (at-r minnie agv_3713_-6384_0))
        (explore-0 :goal (explored pt_obs_613_-8154))
        (:init explore-0 (at-r minnie agv_1763_-8654_0))
        (explore-1 :goal (explored pt_obs_2054_-7404))
        (:init explore-1 (at-r minnie agv_1763_-8654_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-0)

  )
)


(:action patrol_minnie__agv_3713_-6384_0__agv_1763_-8654_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agv_3713_-6384_0))
  :effect (and (explored pt_obs_613_-8154) (explored pt_obs_2054_-7404) (at-r minnie agv_1763_-8654_0))
  
  :methods(

      :method patrol_minnie__agv_3713_-6384_0__agv_1763_-8654_0_i_0
      :actions (move-0 (move minnie agv_3713_-6384_0 agv_2733_-7504_0))
        (move-1 (move minnie agv_2733_-7504_0 agv_1763_-8654_0))
        (explore-0 (observe minnie agv_1763_-8654_0 pt_obs_613_-8154))
        (explore-1 (observe minnie agv_1763_-8654_0 pt_obs_2054_-7404))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agv_3713_-6384_0))
        (move-0 move-1 (at-r minnie agv_2733_-7504_0))
        (move-1 :goal (at-r minnie agv_1763_-8654_0))
        (explore-0 :goal (explored pt_obs_613_-8154))
        (move-1 explore-0 (at-r minnie agv_1763_-8654_0))
        (explore-1 :goal (explored pt_obs_2054_-7404))
        (move-1 explore-1 (at-r minnie agv_1763_-8654_0))
      :temporal-links 

  )
)


(:action patrol_minnie__agv_533_-1973_0__agv_-2186_-4073_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agv_533_-1973_0))
  :effect (and (explored pt_obs_-1486_-3573) (at-r minnie agv_-2186_-4073_0))
  
  :methods(

      :method patrol_minnie__agv_533_-1973_0__agv_-2186_-4073_0_d_3
      :actions (move-0 (move minnie agv_533_-1973_0 agv_-876_-3163_0))
        (move-1 (move minnie agv_-876_-3163_0 agv_-2186_-4073_0))
        (explore-0 (observe minnie agv_-876_-3163_0 pt_obs_-1486_-3573))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agv_533_-1973_0))
        (move-0 move-1 (at-r minnie agv_-876_-3163_0))
        (move-1 :goal (at-r minnie agv_-2186_-4073_0))
        (explore-0 :goal (explored pt_obs_-1486_-3573))
        (move-0 explore-0 (at-r minnie agv_-876_-3163_0))
      :temporal-links (explore-0 move-1)

  )
)


(:action patrol_minnie__agv_-2186_-4073_0__agv_533_-1973_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agv_-2186_-4073_0))
  :effect (and (explored pt_obs_-1486_-3573) (at-r minnie agv_533_-1973_0))
  
  :methods(

      :method patrol_minnie__agv_-2186_-4073_0__agv_533_-1973_0_i_3
      :actions (move-0 (move minnie agv_-2186_-4073_0 agv_-876_-3163_0))
        (move-1 (move minnie agv_-876_-3163_0 agv_533_-1973_0))
        (explore-0 (observe minnie agv_-876_-3163_0 pt_obs_-1486_-3573))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agv_-2186_-4073_0))
        (move-0 move-1 (at-r minnie agv_-876_-3163_0))
        (move-1 :goal (at-r minnie agv_533_-1973_0))
        (explore-0 :goal (explored pt_obs_-1486_-3573))
        (move-0 explore-0 (at-r minnie agv_-876_-3163_0))
      :temporal-links (explore-0 move-1)

  )
)


(:action patrol_minnie__agv_-876_-3163_0__agv_2733_-7504_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agv_-876_-3163_0))
  :effect (and (explored pt_obs_-1486_-3573) (explored pt_obs_2054_-7404) (at-r minnie agv_2733_-7504_0))
  
  :methods(

      :method patrol_minnie__agv_-876_-3163_0__agv_2733_-7504_0_d_2
      :actions (move-0 (move minnie agv_-876_-3163_0 agv_713_-5404_0))
        (move-1 (move minnie agv_713_-5404_0 agv_2733_-7504_0))
        (explore-0 (observe minnie agv_-876_-3163_0 pt_obs_-1486_-3573))
        (explore-1 (observe minnie agv_2733_-7504_0 pt_obs_2054_-7404))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agv_-876_-3163_0))
        (move-0 move-1 (at-r minnie agv_713_-5404_0))
        (move-1 :goal (at-r minnie agv_2733_-7504_0))
        (explore-0 :goal (explored pt_obs_-1486_-3573))
        (:init explore-0 (at-r minnie agv_-876_-3163_0))
        (explore-1 :goal (explored pt_obs_2054_-7404))
        (move-1 explore-1 (at-r minnie agv_2733_-7504_0))
      :temporal-links (explore-0 move-0)

  )
)


(:action patrol_minnie__agv_2733_-7504_0__agv_-876_-3163_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agv_2733_-7504_0))
  :effect (and (explored pt_obs_-1486_-3573) (explored pt_obs_2054_-7404) (at-r minnie agv_-876_-3163_0))
  
  :methods(

      :method patrol_minnie__agv_2733_-7504_0__agv_-876_-3163_0_i_2
      :actions (move-0 (move minnie agv_2733_-7504_0 agv_713_-5404_0))
        (move-1 (move minnie agv_713_-5404_0 agv_-876_-3163_0))
        (explore-0 (observe minnie agv_2733_-7504_0 pt_obs_2054_-7404))
        (explore-1 (observe minnie agv_-876_-3163_0 pt_obs_-1486_-3573))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agv_2733_-7504_0))
        (move-0 move-1 (at-r minnie agv_713_-5404_0))
        (move-1 :goal (at-r minnie agv_-876_-3163_0))
        (explore-0 :goal (explored pt_obs_2054_-7404))
        (:init explore-0 (at-r minnie agv_2733_-7504_0))
        (explore-1 :goal (explored pt_obs_-1486_-3573))
        (move-1 explore-1 (at-r minnie agv_-876_-3163_0))
      :temporal-links (explore-0 move-0)

  )
)


(:action patrol_minnie__agv_3713_-6384_0__agv_4623_-863_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agv_3713_-6384_0))
  :effect (and (explored pt_obs_4154_-4043) (explored pt_obs_4563_-1193) (at-r minnie agv_4623_-863_0))
  
  :methods(

      :method patrol_minnie__agv_3713_-6384_0__agv_4623_-863_0_d_5
      :actions (move-0 (move minnie agv_3713_-6384_0 agv_4183_-3793_0))
        (move-1 (move minnie agv_4183_-3793_0 agv_4623_-863_0))
        (explore-0 (observe minnie agv_4183_-3793_0 pt_obs_4154_-4043))
        (explore-1 (observe minnie agv_4623_-863_0 pt_obs_4563_-1193))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agv_3713_-6384_0))
        (move-0 move-1 (at-r minnie agv_4183_-3793_0))
        (move-1 :goal (at-r minnie agv_4623_-863_0))
        (explore-0 :goal (explored pt_obs_4154_-4043))
        (move-0 explore-0 (at-r minnie agv_4183_-3793_0))
        (explore-1 :goal (explored pt_obs_4563_-1193))
        (move-1 explore-1 (at-r minnie agv_4623_-863_0))
      :temporal-links (explore-0 move-1)

  )
)


(:action patrol_minnie__agv_4623_-863_0__agv_3713_-6384_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agv_4623_-863_0))
  :effect (and (explored pt_obs_4154_-4043) (explored pt_obs_4563_-1193) (at-r minnie agv_3713_-6384_0))
  
  :methods(

      :method patrol_minnie__agv_4623_-863_0__agv_3713_-6384_0_i_5
      :actions (move-0 (move minnie agv_4623_-863_0 agv_4183_-3793_0))
        (move-1 (move minnie agv_4183_-3793_0 agv_3713_-6384_0))
        (explore-0 (observe minnie agv_4623_-863_0 pt_obs_4563_-1193))
        (explore-1 (observe minnie agv_4183_-3793_0 pt_obs_4154_-4043))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agv_4623_-863_0))
        (move-0 move-1 (at-r minnie agv_4183_-3793_0))
        (move-1 :goal (at-r minnie agv_3713_-6384_0))
        (explore-0 :goal (explored pt_obs_4563_-1193))
        (:init explore-0 (at-r minnie agv_4623_-863_0))
        (explore-1 :goal (explored pt_obs_4154_-4043))
        (move-0 explore-1 (at-r minnie agv_4183_-3793_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)

  )
)


(:action patrol_minnie__agv_-26_-163_0__agv_4623_-863_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agv_-26_-163_0))
  :effect (and (explored pt_obs_-606_-223) (explored pt_obs_4563_-1193) (at-r minnie agv_4623_-863_0))
  
  :methods(

      :method patrol_minnie__agv_-26_-163_0__agv_4623_-863_0_d_4
      :actions (move-0 (move minnie agv_-26_-163_0 agv_1533_445_0))
        (move-1 (move minnie agv_1533_445_0 agv_3043_-173_0))
        (move-2 (move minnie agv_3043_-173_0 agv_4004_286_0))
        (move-3 (move minnie agv_4004_286_0 agv_4623_-863_0))
        (explore-0 (observe minnie agv_-26_-163_0 pt_obs_-606_-223))
        (explore-1 (observe minnie agv_4623_-863_0 pt_obs_4563_-1193))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agv_-26_-163_0))
        (move-0 move-1 (at-r minnie agv_1533_445_0))
        (move-1 move-2 (at-r minnie agv_3043_-173_0))
        (move-2 move-3 (at-r minnie agv_4004_286_0))
        (move-3 :goal (at-r minnie agv_4623_-863_0))
        (explore-0 :goal (explored pt_obs_-606_-223))
        (:init explore-0 (at-r minnie agv_-26_-163_0))
        (explore-1 :goal (explored pt_obs_4563_-1193))
        (move-3 explore-1 (at-r minnie agv_4623_-863_0))
      :temporal-links (explore-0 move-0)

  )
)


(:action patrol_minnie__agv_4623_-863_0__agv_-26_-163_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agv_4623_-863_0))
  :effect (and (explored pt_obs_-606_-223) (explored pt_obs_4563_-1193) (at-r minnie agv_-26_-163_0))
  
  :methods(

      :method patrol_minnie__agv_4623_-863_0__agv_-26_-163_0_i_4
      :actions (move-0 (move minnie agv_4623_-863_0 agv_4004_286_0))
        (move-1 (move minnie agv_4004_286_0 agv_3043_-173_0))
        (move-2 (move minnie agv_3043_-173_0 agv_1533_445_0))
        (move-3 (move minnie agv_1533_445_0 agv_-26_-163_0))
        (explore-0 (observe minnie agv_4623_-863_0 pt_obs_4563_-1193))
        (explore-1 (observe minnie agv_-26_-163_0 pt_obs_-606_-223))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agv_4623_-863_0))
        (move-0 move-1 (at-r minnie agv_4004_286_0))
        (move-1 move-2 (at-r minnie agv_3043_-173_0))
        (move-2 move-3 (at-r minnie agv_1533_445_0))
        (move-3 :goal (at-r minnie agv_-26_-163_0))
        (explore-0 :goal (explored pt_obs_4563_-1193))
        (:init explore-0 (at-r minnie agv_4623_-863_0))
        (explore-1 :goal (explored pt_obs_-606_-223))
        (move-3 explore-1 (at-r minnie agv_-26_-163_0))
      :temporal-links (explore-0 move-0)

  )
)


(:action patrol_minnie__agv_1323_-3473_0__agv_4623_-863_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agv_1323_-3473_0))
  :effect (and (explored pt_obs_1593_-3273) (explored pt_obs_4563_-1193) (at-r minnie agv_4623_-863_0))
  
  :methods(

      :method patrol_minnie__agv_1323_-3473_0__agv_4623_-863_0_d_7
      :actions (move-0 (move minnie agv_1323_-3473_0 agv_4623_-863_0))
        (explore-0 (observe minnie agv_1323_-3473_0 pt_obs_1593_-3273))
        (explore-1 (observe minnie agv_4623_-863_0 pt_obs_4563_-1193))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agv_1323_-3473_0))
        (move-0 :goal (at-r minnie agv_4623_-863_0))
        (explore-0 :goal (explored pt_obs_1593_-3273))
        (:init explore-0 (at-r minnie agv_1323_-3473_0))
        (explore-1 :goal (explored pt_obs_4563_-1193))
        (move-0 explore-1 (at-r minnie agv_4623_-863_0))
      :temporal-links (explore-0 move-0)

  )
)


(:action patrol_minnie__agv_4623_-863_0__agv_1323_-3473_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agv_4623_-863_0))
  :effect (and (explored pt_obs_1593_-3273) (explored pt_obs_4563_-1193) (at-r minnie agv_1323_-3473_0))
  
  :methods(

      :method patrol_minnie__agv_4623_-863_0__agv_1323_-3473_0_i_7
      :actions (move-0 (move minnie agv_4623_-863_0 agv_1323_-3473_0))
        (explore-0 (observe minnie agv_4623_-863_0 pt_obs_4563_-1193))
        (explore-1 (observe minnie agv_1323_-3473_0 pt_obs_1593_-3273))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agv_4623_-863_0))
        (move-0 :goal (at-r minnie agv_1323_-3473_0))
        (explore-0 :goal (explored pt_obs_4563_-1193))
        (:init explore-0 (at-r minnie agv_4623_-863_0))
        (explore-1 :goal (explored pt_obs_1593_-3273))
        (move-0 explore-1 (at-r minnie agv_1323_-3473_0))
      :temporal-links (explore-0 move-0)

  )
)


(:action patrol_minnie__agv_533_-1973_0__agv_3713_-6384_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agv_533_-1973_0))
  :effect (and (explored pt_obs_1593_-3273) (at-r minnie agv_3713_-6384_0))
  
  :methods(

      :method patrol_minnie__agv_533_-1973_0__agv_3713_-6384_0_d_6
      :actions (move-0 (move minnie agv_533_-1973_0 agv_1323_-3473_0))
        (move-1 (move minnie agv_1323_-3473_0 agv_3713_-6384_0))
        (explore-0 (observe minnie agv_1323_-3473_0 pt_obs_1593_-3273))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agv_533_-1973_0))
        (move-0 move-1 (at-r minnie agv_1323_-3473_0))
        (move-1 :goal (at-r minnie agv_3713_-6384_0))
        (explore-0 :goal (explored pt_obs_1593_-3273))
        (move-0 explore-0 (at-r minnie agv_1323_-3473_0))
      :temporal-links (explore-0 move-1)

  )
)


(:action patrol_minnie__agv_3713_-6384_0__agv_533_-1973_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agv_3713_-6384_0))
  :effect (and (explored pt_obs_1593_-3273) (at-r minnie agv_533_-1973_0))
  
  :methods(

      :method patrol_minnie__agv_3713_-6384_0__agv_533_-1973_0_i_6
      :actions (move-0 (move minnie agv_3713_-6384_0 agv_1323_-3473_0))
        (move-1 (move minnie agv_1323_-3473_0 agv_533_-1973_0))
        (explore-0 (observe minnie agv_1323_-3473_0 pt_obs_1593_-3273))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agv_3713_-6384_0))
        (move-0 move-1 (at-r minnie agv_1323_-3473_0))
        (move-1 :goal (at-r minnie agv_533_-1973_0))
        (explore-0 :goal (explored pt_obs_1593_-3273))
        (move-0 explore-0 (at-r minnie agv_1323_-3473_0))
      :temporal-links (explore-0 move-1)

  )
)

)
