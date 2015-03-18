
(define (domain-helper action-V)
    (:options :abstractOnly :erasePlansWhenAbstractMet)
    (:allowed-actions move-agv move-aav communicate-aav-agv communicate-aav-aav has-communicated)
    (:low-priority-predicates )

    
(:action patrolagv__pt_agv_7764_1138__pt_agv_17037_6506
  :parameters (?r - agv)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_agv_7764_1138))
  :effect (and (explored pt_obs_10249_3401) (explored pt_obs_16239_5397) (explored pt_obs_8252_3401) (explored pt_obs_10249_1404) (explored pt_obs_14242_5397) (explored pt_obs_12245_1404) (explored pt_obs_18235_5397) (explored pt_obs_8252_1404) (at-r ?r pt_agv_17037_6506))
  
  :methods(

      :method direct
      :actions (move-0 (move-agv ?r pt_agv_7764_1138 pt_agv_8740_1626))
        (move-1 (move-agv ?r pt_agv_8740_1626 pt_agv_11669_2602))
        (move-2 (move-agv ?r pt_agv_11669_2602 pt_agv_14109_4066))
        (move-3 (move-agv ?r pt_agv_14109_4066 pt_agv_15085_5042))
        (move-4 (move-agv ?r pt_agv_15085_5042 pt_agv_17037_6506))
        (explore-0 (observe-agv ?r pt_agv_8740_1626 pt_obs_8252_3401))
        (explore-1 (observe-agv ?r pt_agv_8740_1626 pt_obs_8252_1404))
        (explore-2 (observe-agv ?r pt_agv_11669_2602 pt_obs_10249_3401))
        (explore-3 (observe-agv ?r pt_agv_11669_2602 pt_obs_10249_1404))
        (explore-4 (observe-agv ?r pt_agv_11669_2602 pt_obs_12245_1404))
        (explore-5 (observe-agv ?r pt_agv_15085_5042 pt_obs_14242_5397))
        (explore-6 (observe-agv ?r pt_agv_15085_5042 pt_obs_16239_5397))
        (explore-7 (observe-agv ?r pt_agv_17037_6506 pt_obs_18235_5397))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_agv_7764_1138))
        (move-0 move-1 (at-r ?r pt_agv_8740_1626))
        (move-1 move-2 (at-r ?r pt_agv_11669_2602))
        (move-2 move-3 (at-r ?r pt_agv_14109_4066))
        (move-3 move-4 (at-r ?r pt_agv_15085_5042))
        (move-4 :goal (at-r ?r pt_agv_17037_6506))
        (explore-0 :goal (explored pt_obs_8252_3401))
        (move-0 explore-0 (at-r ?r pt_agv_8740_1626))
        (explore-1 :goal (explored pt_obs_8252_1404))
        (move-0 explore-1 (at-r ?r pt_agv_8740_1626))
        (explore-2 :goal (explored pt_obs_10249_3401))
        (move-1 explore-2 (at-r ?r pt_agv_11669_2602))
        (explore-3 :goal (explored pt_obs_10249_1404))
        (move-1 explore-3 (at-r ?r pt_agv_11669_2602))
        (explore-4 :goal (explored pt_obs_12245_1404))
        (move-1 explore-4 (at-r ?r pt_agv_11669_2602))
        (explore-5 :goal (explored pt_obs_14242_5397))
        (move-3 explore-5 (at-r ?r pt_agv_15085_5042))
        (explore-6 :goal (explored pt_obs_16239_5397))
        (move-3 explore-6 (at-r ?r pt_agv_15085_5042))
        (explore-7 :goal (explored pt_obs_18235_5397))
        (move-4 explore-7 (at-r ?r pt_agv_17037_6506))
      :temporal-links (explore-0 move-1)
        (explore-1 move-1)
        (explore-2 move-2)
        (explore-3 move-2)
        (explore-4 move-2)
        (explore-5 move-4)
        (explore-6 move-4)

  )
)


(:action patrolagv__pt_agv_17037_6506__pt_agv_7764_1138
  :parameters (?r - agv)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_agv_17037_6506))
  :effect (and (explored pt_obs_10249_3401) (explored pt_obs_16239_5397) (explored pt_obs_8252_3401) (explored pt_obs_10249_1404) (explored pt_obs_14242_5397) (explored pt_obs_12245_1404) (explored pt_obs_18235_5397) (explored pt_obs_8252_1404) (at-r ?r pt_agv_7764_1138))
  
  :methods(

      :method direct
      :actions (move-0 (move-agv ?r pt_agv_17037_6506 pt_agv_15085_5042))
        (move-1 (move-agv ?r pt_agv_15085_5042 pt_agv_14109_4066))
        (move-2 (move-agv ?r pt_agv_14109_4066 pt_agv_11669_2602))
        (move-3 (move-agv ?r pt_agv_11669_2602 pt_agv_8740_1626))
        (move-4 (move-agv ?r pt_agv_8740_1626 pt_agv_7764_1138))
        (explore-0 (observe-agv ?r pt_agv_17037_6506 pt_obs_18235_5397))
        (explore-1 (observe-agv ?r pt_agv_15085_5042 pt_obs_14242_5397))
        (explore-2 (observe-agv ?r pt_agv_15085_5042 pt_obs_16239_5397))
        (explore-3 (observe-agv ?r pt_agv_11669_2602 pt_obs_10249_3401))
        (explore-4 (observe-agv ?r pt_agv_11669_2602 pt_obs_10249_1404))
        (explore-5 (observe-agv ?r pt_agv_11669_2602 pt_obs_12245_1404))
        (explore-6 (observe-agv ?r pt_agv_8740_1626 pt_obs_8252_3401))
        (explore-7 (observe-agv ?r pt_agv_8740_1626 pt_obs_8252_1404))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_agv_17037_6506))
        (move-0 move-1 (at-r ?r pt_agv_15085_5042))
        (move-1 move-2 (at-r ?r pt_agv_14109_4066))
        (move-2 move-3 (at-r ?r pt_agv_11669_2602))
        (move-3 move-4 (at-r ?r pt_agv_8740_1626))
        (move-4 :goal (at-r ?r pt_agv_7764_1138))
        (explore-0 :goal (explored pt_obs_18235_5397))
        (:init explore-0 (at-r ?r pt_agv_17037_6506))
        (explore-1 :goal (explored pt_obs_14242_5397))
        (move-0 explore-1 (at-r ?r pt_agv_15085_5042))
        (explore-2 :goal (explored pt_obs_16239_5397))
        (move-0 explore-2 (at-r ?r pt_agv_15085_5042))
        (explore-3 :goal (explored pt_obs_10249_3401))
        (move-2 explore-3 (at-r ?r pt_agv_11669_2602))
        (explore-4 :goal (explored pt_obs_10249_1404))
        (move-2 explore-4 (at-r ?r pt_agv_11669_2602))
        (explore-5 :goal (explored pt_obs_12245_1404))
        (move-2 explore-5 (at-r ?r pt_agv_11669_2602))
        (explore-6 :goal (explored pt_obs_8252_3401))
        (move-3 explore-6 (at-r ?r pt_agv_8740_1626))
        (explore-7 :goal (explored pt_obs_8252_1404))
        (move-3 explore-7 (at-r ?r pt_agv_8740_1626))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)
        (explore-2 move-1)
        (explore-3 move-3)
        (explore-4 move-3)
        (explore-5 move-3)
        (explore-6 move-4)
        (explore-7 move-4)

  )
)


(:action patrolagv__pt_agv_23382_-1790__pt_agv_17037_6506
  :parameters (?r - agv)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_agv_23382_-1790))
  :effect (and (explored pt_obs_20232_1404) (explored pt_obs_22229_1404) (explored pt_obs_22229_3401) (explored pt_obs_20232_3401) (explored pt_obs_18235_3401) (explored pt_obs_18235_5397) (explored pt_obs_22229_-592) (explored pt_obs_20232_5397) (at-r ?r pt_agv_17037_6506))
  
  :methods(

      :method direct
      :actions (move-0 (move-agv ?r pt_agv_23382_-1790 pt_agv_22406_1138))
        (move-1 (move-agv ?r pt_agv_22406_1138 pt_agv_20942_2602))
        (move-2 (move-agv ?r pt_agv_20942_2602 pt_agv_19478_4554))
        (move-3 (move-agv ?r pt_agv_19478_4554 pt_agv_18502_5530))
        (move-4 (move-agv ?r pt_agv_18502_5530 pt_agv_17037_6506))
        (explore-0 (observe-agv ?r pt_agv_22406_1138 pt_obs_22229_-592))
        (explore-1 (observe-agv ?r pt_agv_20942_2602 pt_obs_20232_1404))
        (explore-2 (observe-agv ?r pt_agv_20942_2602 pt_obs_22229_3401))
        (explore-3 (observe-agv ?r pt_agv_20942_2602 pt_obs_22229_1404))
        (explore-4 (observe-agv ?r pt_agv_19478_4554 pt_obs_18235_5397))
        (explore-5 (observe-agv ?r pt_agv_19478_4554 pt_obs_18235_3401))
        (explore-6 (observe-agv ?r pt_agv_19478_4554 pt_obs_20232_5397))
        (explore-7 (observe-agv ?r pt_agv_19478_4554 pt_obs_20232_3401))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_agv_23382_-1790))
        (move-0 move-1 (at-r ?r pt_agv_22406_1138))
        (move-1 move-2 (at-r ?r pt_agv_20942_2602))
        (move-2 move-3 (at-r ?r pt_agv_19478_4554))
        (move-3 move-4 (at-r ?r pt_agv_18502_5530))
        (move-4 :goal (at-r ?r pt_agv_17037_6506))
        (explore-0 :goal (explored pt_obs_22229_-592))
        (move-0 explore-0 (at-r ?r pt_agv_22406_1138))
        (explore-1 :goal (explored pt_obs_20232_1404))
        (move-1 explore-1 (at-r ?r pt_agv_20942_2602))
        (explore-2 :goal (explored pt_obs_22229_3401))
        (move-1 explore-2 (at-r ?r pt_agv_20942_2602))
        (explore-3 :goal (explored pt_obs_22229_1404))
        (move-1 explore-3 (at-r ?r pt_agv_20942_2602))
        (explore-4 :goal (explored pt_obs_18235_5397))
        (move-2 explore-4 (at-r ?r pt_agv_19478_4554))
        (explore-5 :goal (explored pt_obs_18235_3401))
        (move-2 explore-5 (at-r ?r pt_agv_19478_4554))
        (explore-6 :goal (explored pt_obs_20232_5397))
        (move-2 explore-6 (at-r ?r pt_agv_19478_4554))
        (explore-7 :goal (explored pt_obs_20232_3401))
        (move-2 explore-7 (at-r ?r pt_agv_19478_4554))
      :temporal-links (explore-0 move-1)
        (explore-1 move-2)
        (explore-2 move-2)
        (explore-3 move-2)
        (explore-4 move-3)
        (explore-5 move-3)
        (explore-6 move-3)
        (explore-7 move-3)

  )
)


(:action patrolagv__pt_agv_17037_6506__pt_agv_23382_-1790
  :parameters (?r - agv)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_agv_17037_6506))
  :effect (and (explored pt_obs_20232_1404) (explored pt_obs_22229_1404) (explored pt_obs_22229_3401) (explored pt_obs_20232_3401) (explored pt_obs_18235_3401) (explored pt_obs_18235_5397) (explored pt_obs_22229_-592) (explored pt_obs_20232_5397) (at-r ?r pt_agv_23382_-1790))
  
  :methods(

      :method direct
      :actions (move-0 (move-agv ?r pt_agv_17037_6506 pt_agv_18502_5530))
        (move-1 (move-agv ?r pt_agv_18502_5530 pt_agv_19478_4554))
        (move-2 (move-agv ?r pt_agv_19478_4554 pt_agv_20942_2602))
        (move-3 (move-agv ?r pt_agv_20942_2602 pt_agv_22406_1138))
        (move-4 (move-agv ?r pt_agv_22406_1138 pt_agv_23382_-1790))
        (explore-0 (observe-agv ?r pt_agv_18502_5530 pt_obs_20232_5397))
        (explore-1 (observe-agv ?r pt_agv_19478_4554 pt_obs_18235_5397))
        (explore-2 (observe-agv ?r pt_agv_19478_4554 pt_obs_18235_3401))
        (explore-3 (observe-agv ?r pt_agv_19478_4554 pt_obs_20232_3401))
        (explore-4 (observe-agv ?r pt_agv_20942_2602 pt_obs_20232_1404))
        (explore-5 (observe-agv ?r pt_agv_20942_2602 pt_obs_22229_3401))
        (explore-6 (observe-agv ?r pt_agv_20942_2602 pt_obs_22229_1404))
        (explore-7 (observe-agv ?r pt_agv_22406_1138 pt_obs_22229_-592))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_agv_17037_6506))
        (move-0 move-1 (at-r ?r pt_agv_18502_5530))
        (move-1 move-2 (at-r ?r pt_agv_19478_4554))
        (move-2 move-3 (at-r ?r pt_agv_20942_2602))
        (move-3 move-4 (at-r ?r pt_agv_22406_1138))
        (move-4 :goal (at-r ?r pt_agv_23382_-1790))
        (explore-0 :goal (explored pt_obs_20232_5397))
        (move-0 explore-0 (at-r ?r pt_agv_18502_5530))
        (explore-1 :goal (explored pt_obs_18235_5397))
        (move-1 explore-1 (at-r ?r pt_agv_19478_4554))
        (explore-2 :goal (explored pt_obs_18235_3401))
        (move-1 explore-2 (at-r ?r pt_agv_19478_4554))
        (explore-3 :goal (explored pt_obs_20232_3401))
        (move-1 explore-3 (at-r ?r pt_agv_19478_4554))
        (explore-4 :goal (explored pt_obs_20232_1404))
        (move-2 explore-4 (at-r ?r pt_agv_20942_2602))
        (explore-5 :goal (explored pt_obs_22229_3401))
        (move-2 explore-5 (at-r ?r pt_agv_20942_2602))
        (explore-6 :goal (explored pt_obs_22229_1404))
        (move-2 explore-6 (at-r ?r pt_agv_20942_2602))
        (explore-7 :goal (explored pt_obs_22229_-592))
        (move-3 explore-7 (at-r ?r pt_agv_22406_1138))
      :temporal-links (explore-0 move-1)
        (explore-1 move-2)
        (explore-2 move-2)
        (explore-3 move-2)
        (explore-4 move-3)
        (explore-5 move-3)
        (explore-6 move-3)
        (explore-7 move-4)

  )
)


(:action patrolagv__pt_agv_16061_-10087__pt_agv_23382_-1790
  :parameters (?r - agv)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_agv_16061_-10087))
  :effect (and (explored pt_obs_20232_-4585) (explored pt_obs_22229_-4585) (explored pt_obs_20232_-2588) (explored pt_obs_20232_-6582) (explored pt_obs_16239_-10575) (explored pt_obs_18235_-8578) (at-r ?r pt_agv_23382_-1790))
  
  :methods(

      :method direct
      :actions (move-0 (move-agv ?r pt_agv_16061_-10087 pt_agv_17525_-8623))
        (move-1 (move-agv ?r pt_agv_17525_-8623 pt_agv_20454_-5206))
        (move-2 (move-agv ?r pt_agv_20454_-5206 pt_agv_20942_-4230))
        (move-3 (move-agv ?r pt_agv_20942_-4230 pt_agv_23382_-1790))
        (explore-0 (observe-agv ?r pt_agv_16061_-10087 pt_obs_16239_-10575))
        (explore-1 (observe-agv ?r pt_agv_17525_-8623 pt_obs_18235_-8578))
        (explore-2 (observe-agv ?r pt_agv_20454_-5206 pt_obs_20232_-6582))
        (explore-3 (observe-agv ?r pt_agv_20942_-4230 pt_obs_20232_-2588))
        (explore-4 (observe-agv ?r pt_agv_20942_-4230 pt_obs_20232_-4585))
        (explore-5 (observe-agv ?r pt_agv_20942_-4230 pt_obs_22229_-4585))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_agv_16061_-10087))
        (move-0 move-1 (at-r ?r pt_agv_17525_-8623))
        (move-1 move-2 (at-r ?r pt_agv_20454_-5206))
        (move-2 move-3 (at-r ?r pt_agv_20942_-4230))
        (move-3 :goal (at-r ?r pt_agv_23382_-1790))
        (explore-0 :goal (explored pt_obs_16239_-10575))
        (:init explore-0 (at-r ?r pt_agv_16061_-10087))
        (explore-1 :goal (explored pt_obs_18235_-8578))
        (move-0 explore-1 (at-r ?r pt_agv_17525_-8623))
        (explore-2 :goal (explored pt_obs_20232_-6582))
        (move-1 explore-2 (at-r ?r pt_agv_20454_-5206))
        (explore-3 :goal (explored pt_obs_20232_-2588))
        (move-2 explore-3 (at-r ?r pt_agv_20942_-4230))
        (explore-4 :goal (explored pt_obs_20232_-4585))
        (move-2 explore-4 (at-r ?r pt_agv_20942_-4230))
        (explore-5 :goal (explored pt_obs_22229_-4585))
        (move-2 explore-5 (at-r ?r pt_agv_20942_-4230))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)
        (explore-2 move-2)
        (explore-3 move-3)
        (explore-4 move-3)
        (explore-5 move-3)

  )
)


(:action patrolagv__pt_agv_23382_-1790__pt_agv_16061_-10087
  :parameters (?r - agv)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_agv_23382_-1790))
  :effect (and (explored pt_obs_20232_-4585) (explored pt_obs_22229_-4585) (explored pt_obs_20232_-2588) (explored pt_obs_20232_-6582) (explored pt_obs_16239_-10575) (explored pt_obs_18235_-8578) (at-r ?r pt_agv_16061_-10087))
  
  :methods(

      :method direct
      :actions (move-0 (move-agv ?r pt_agv_23382_-1790 pt_agv_20942_-4230))
        (move-1 (move-agv ?r pt_agv_20942_-4230 pt_agv_20454_-5206))
        (move-2 (move-agv ?r pt_agv_20454_-5206 pt_agv_17525_-8623))
        (move-3 (move-agv ?r pt_agv_17525_-8623 pt_agv_16061_-10087))
        (explore-0 (observe-agv ?r pt_agv_20942_-4230 pt_obs_20232_-2588))
        (explore-1 (observe-agv ?r pt_agv_20942_-4230 pt_obs_20232_-4585))
        (explore-2 (observe-agv ?r pt_agv_20942_-4230 pt_obs_22229_-4585))
        (explore-3 (observe-agv ?r pt_agv_20454_-5206 pt_obs_20232_-6582))
        (explore-4 (observe-agv ?r pt_agv_17525_-8623 pt_obs_18235_-8578))
        (explore-5 (observe-agv ?r pt_agv_16061_-10087 pt_obs_16239_-10575))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_agv_23382_-1790))
        (move-0 move-1 (at-r ?r pt_agv_20942_-4230))
        (move-1 move-2 (at-r ?r pt_agv_20454_-5206))
        (move-2 move-3 (at-r ?r pt_agv_17525_-8623))
        (move-3 :goal (at-r ?r pt_agv_16061_-10087))
        (explore-0 :goal (explored pt_obs_20232_-2588))
        (move-0 explore-0 (at-r ?r pt_agv_20942_-4230))
        (explore-1 :goal (explored pt_obs_20232_-4585))
        (move-0 explore-1 (at-r ?r pt_agv_20942_-4230))
        (explore-2 :goal (explored pt_obs_22229_-4585))
        (move-0 explore-2 (at-r ?r pt_agv_20942_-4230))
        (explore-3 :goal (explored pt_obs_20232_-6582))
        (move-1 explore-3 (at-r ?r pt_agv_20454_-5206))
        (explore-4 :goal (explored pt_obs_18235_-8578))
        (move-2 explore-4 (at-r ?r pt_agv_17525_-8623))
        (explore-5 :goal (explored pt_obs_16239_-10575))
        (move-3 explore-5 (at-r ?r pt_agv_16061_-10087))
      :temporal-links (explore-0 move-1)
        (explore-1 move-1)
        (explore-2 move-1)
        (explore-3 move-2)
        (explore-4 move-3)

  )
)


(:action patrolagv__pt_agv_16061_-10087__pt_agv_7764_1138
  :parameters (?r - agv)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_agv_16061_-10087))
  :effect (and (explored pt_obs_10249_-2588) (explored pt_obs_10249_-592) (explored pt_obs_14242_-7580) (explored pt_obs_12245_-4585) (explored pt_obs_12745_-5583) (explored pt_obs_8252_1404) (explored pt_obs_16239_-10575) (at-r ?r pt_agv_7764_1138))
  
  :methods(

      :method direct
      :actions (move-0 (move-agv ?r pt_agv_16061_-10087 pt_agv_14109_-7158))
        (move-1 (move-agv ?r pt_agv_14109_-7158 pt_agv_12645_-5206))
        (move-2 (move-agv ?r pt_agv_12645_-5206 pt_agv_11669_-4230))
        (move-3 (move-agv ?r pt_agv_11669_-4230 pt_agv_9228_-1302))
        (move-4 (move-agv ?r pt_agv_9228_-1302 pt_agv_7764_1138))
        (explore-0 (observe-agv ?r pt_agv_16061_-10087 pt_obs_16239_-10575))
        (explore-1 (observe-agv ?r pt_agv_14109_-7158 pt_obs_14242_-7580))
        (explore-2 (observe-agv ?r pt_agv_12645_-5206 pt_obs_12745_-5583))
        (explore-3 (observe-agv ?r pt_agv_11669_-4230 pt_obs_12245_-4585))
        (explore-4 (observe-agv ?r pt_agv_9228_-1302 pt_obs_10249_-592))
        (explore-5 (observe-agv ?r pt_agv_9228_-1302 pt_obs_10249_-2588))
        (explore-6 (observe-agv ?r pt_agv_7764_1138 pt_obs_8252_1404))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_agv_16061_-10087))
        (move-0 move-1 (at-r ?r pt_agv_14109_-7158))
        (move-1 move-2 (at-r ?r pt_agv_12645_-5206))
        (move-2 move-3 (at-r ?r pt_agv_11669_-4230))
        (move-3 move-4 (at-r ?r pt_agv_9228_-1302))
        (move-4 :goal (at-r ?r pt_agv_7764_1138))
        (explore-0 :goal (explored pt_obs_16239_-10575))
        (:init explore-0 (at-r ?r pt_agv_16061_-10087))
        (explore-1 :goal (explored pt_obs_14242_-7580))
        (move-0 explore-1 (at-r ?r pt_agv_14109_-7158))
        (explore-2 :goal (explored pt_obs_12745_-5583))
        (move-1 explore-2 (at-r ?r pt_agv_12645_-5206))
        (explore-3 :goal (explored pt_obs_12245_-4585))
        (move-2 explore-3 (at-r ?r pt_agv_11669_-4230))
        (explore-4 :goal (explored pt_obs_10249_-592))
        (move-3 explore-4 (at-r ?r pt_agv_9228_-1302))
        (explore-5 :goal (explored pt_obs_10249_-2588))
        (move-3 explore-5 (at-r ?r pt_agv_9228_-1302))
        (explore-6 :goal (explored pt_obs_8252_1404))
        (move-4 explore-6 (at-r ?r pt_agv_7764_1138))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)
        (explore-2 move-2)
        (explore-3 move-3)
        (explore-4 move-4)
        (explore-5 move-4)

  )
)


(:action patrolagv__pt_agv_7764_1138__pt_agv_16061_-10087
  :parameters (?r - agv)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_agv_7764_1138))
  :effect (and (explored pt_obs_10249_-2588) (explored pt_obs_10249_-592) (explored pt_obs_14242_-7580) (explored pt_obs_12245_-4585) (explored pt_obs_12745_-5583) (explored pt_obs_8252_1404) (explored pt_obs_16239_-10575) (at-r ?r pt_agv_16061_-10087))
  
  :methods(

      :method direct
      :actions (move-0 (move-agv ?r pt_agv_7764_1138 pt_agv_9228_-1302))
        (move-1 (move-agv ?r pt_agv_9228_-1302 pt_agv_11669_-4230))
        (move-2 (move-agv ?r pt_agv_11669_-4230 pt_agv_12645_-5206))
        (move-3 (move-agv ?r pt_agv_12645_-5206 pt_agv_14109_-7158))
        (move-4 (move-agv ?r pt_agv_14109_-7158 pt_agv_16061_-10087))
        (explore-0 (observe-agv ?r pt_agv_7764_1138 pt_obs_8252_1404))
        (explore-1 (observe-agv ?r pt_agv_9228_-1302 pt_obs_10249_-592))
        (explore-2 (observe-agv ?r pt_agv_9228_-1302 pt_obs_10249_-2588))
        (explore-3 (observe-agv ?r pt_agv_11669_-4230 pt_obs_12245_-4585))
        (explore-4 (observe-agv ?r pt_agv_12645_-5206 pt_obs_12745_-5583))
        (explore-5 (observe-agv ?r pt_agv_14109_-7158 pt_obs_14242_-7580))
        (explore-6 (observe-agv ?r pt_agv_16061_-10087 pt_obs_16239_-10575))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_agv_7764_1138))
        (move-0 move-1 (at-r ?r pt_agv_9228_-1302))
        (move-1 move-2 (at-r ?r pt_agv_11669_-4230))
        (move-2 move-3 (at-r ?r pt_agv_12645_-5206))
        (move-3 move-4 (at-r ?r pt_agv_14109_-7158))
        (move-4 :goal (at-r ?r pt_agv_16061_-10087))
        (explore-0 :goal (explored pt_obs_8252_1404))
        (:init explore-0 (at-r ?r pt_agv_7764_1138))
        (explore-1 :goal (explored pt_obs_10249_-592))
        (move-0 explore-1 (at-r ?r pt_agv_9228_-1302))
        (explore-2 :goal (explored pt_obs_10249_-2588))
        (move-0 explore-2 (at-r ?r pt_agv_9228_-1302))
        (explore-3 :goal (explored pt_obs_12245_-4585))
        (move-1 explore-3 (at-r ?r pt_agv_11669_-4230))
        (explore-4 :goal (explored pt_obs_12745_-5583))
        (move-2 explore-4 (at-r ?r pt_agv_12645_-5206))
        (explore-5 :goal (explored pt_obs_14242_-7580))
        (move-3 explore-5 (at-r ?r pt_agv_14109_-7158))
        (explore-6 :goal (explored pt_obs_16239_-10575))
        (move-4 explore-6 (at-r ?r pt_agv_16061_-10087))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)
        (explore-2 move-1)
        (explore-3 move-2)
        (explore-4 move-3)
        (explore-5 move-4)

  )
)


(:action patrolaav__pt_aav_14242_-2588__pt_aav_12245_-6582
  :parameters (?r - aav)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_aav_14242_-2588))
  :effect (and (explored pt_obs_12245_-6582) (explored pt_obs_14242_-4585) (explored pt_obs_14242_-2588) (explored pt_obs_12245_-8578) (explored pt_obs_14242_-6582) (explored pt_obs_14242_-8578) (at-r ?r pt_aav_12245_-6582))
  
  :methods(

      :method direct
      :actions (move-0 (move-aav ?r pt_aav_14242_-2588 pt_aav_14242_-4585))
        (move-1 (move-aav ?r pt_aav_14242_-4585 pt_aav_14242_-6582))
        (move-2 (move-aav ?r pt_aav_14242_-6582 pt_aav_14242_-8578))
        (move-3 (move-aav ?r pt_aav_14242_-8578 pt_aav_12245_-8578))
        (move-4 (move-aav ?r pt_aav_12245_-8578 pt_aav_12245_-6582))
        (explore-0 (observe-aav ?r pt_aav_14242_-2588 pt_obs_14242_-2588))
        (explore-1 (observe-aav ?r pt_aav_14242_-4585 pt_obs_14242_-4585))
        (explore-2 (observe-aav ?r pt_aav_14242_-6582 pt_obs_14242_-6582))
        (explore-3 (observe-aav ?r pt_aav_14242_-8578 pt_obs_14242_-8578))
        (explore-4 (observe-aav ?r pt_aav_12245_-8578 pt_obs_12245_-8578))
        (explore-5 (observe-aav ?r pt_aav_12245_-6582 pt_obs_12245_-6582))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_aav_14242_-2588))
        (move-0 move-1 (at-r ?r pt_aav_14242_-4585))
        (move-1 move-2 (at-r ?r pt_aav_14242_-6582))
        (move-2 move-3 (at-r ?r pt_aav_14242_-8578))
        (move-3 move-4 (at-r ?r pt_aav_12245_-8578))
        (move-4 :goal (at-r ?r pt_aav_12245_-6582))
        (explore-0 :goal (explored pt_obs_14242_-2588))
        (:init explore-0 (at-r ?r pt_aav_14242_-2588))
        (explore-1 :goal (explored pt_obs_14242_-4585))
        (move-0 explore-1 (at-r ?r pt_aav_14242_-4585))
        (explore-2 :goal (explored pt_obs_14242_-6582))
        (move-1 explore-2 (at-r ?r pt_aav_14242_-6582))
        (explore-3 :goal (explored pt_obs_14242_-8578))
        (move-2 explore-3 (at-r ?r pt_aav_14242_-8578))
        (explore-4 :goal (explored pt_obs_12245_-8578))
        (move-3 explore-4 (at-r ?r pt_aav_12245_-8578))
        (explore-5 :goal (explored pt_obs_12245_-6582))
        (move-4 explore-5 (at-r ?r pt_aav_12245_-6582))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)
        (explore-2 move-2)
        (explore-3 move-3)
        (explore-4 move-4)

  )
)


(:action patrolaav__pt_aav_12245_-6582__pt_aav_14242_-2588
  :parameters (?r - aav)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_aav_12245_-6582))
  :effect (and (explored pt_obs_12245_-6582) (explored pt_obs_14242_-4585) (explored pt_obs_14242_-2588) (explored pt_obs_12245_-8578) (explored pt_obs_14242_-6582) (explored pt_obs_14242_-8578) (at-r ?r pt_aav_14242_-2588))
  
  :methods(

      :method direct
      :actions (move-0 (move-aav ?r pt_aav_12245_-6582 pt_aav_12245_-8578))
        (move-1 (move-aav ?r pt_aav_12245_-8578 pt_aav_14242_-8578))
        (move-2 (move-aav ?r pt_aav_14242_-8578 pt_aav_14242_-6582))
        (move-3 (move-aav ?r pt_aav_14242_-6582 pt_aav_14242_-4585))
        (move-4 (move-aav ?r pt_aav_14242_-4585 pt_aav_14242_-2588))
        (explore-0 (observe-aav ?r pt_aav_12245_-6582 pt_obs_12245_-6582))
        (explore-1 (observe-aav ?r pt_aav_12245_-8578 pt_obs_12245_-8578))
        (explore-2 (observe-aav ?r pt_aav_14242_-8578 pt_obs_14242_-8578))
        (explore-3 (observe-aav ?r pt_aav_14242_-6582 pt_obs_14242_-6582))
        (explore-4 (observe-aav ?r pt_aav_14242_-4585 pt_obs_14242_-4585))
        (explore-5 (observe-aav ?r pt_aav_14242_-2588 pt_obs_14242_-2588))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_aav_12245_-6582))
        (move-0 move-1 (at-r ?r pt_aav_12245_-8578))
        (move-1 move-2 (at-r ?r pt_aav_14242_-8578))
        (move-2 move-3 (at-r ?r pt_aav_14242_-6582))
        (move-3 move-4 (at-r ?r pt_aav_14242_-4585))
        (move-4 :goal (at-r ?r pt_aav_14242_-2588))
        (explore-0 :goal (explored pt_obs_12245_-6582))
        (:init explore-0 (at-r ?r pt_aav_12245_-6582))
        (explore-1 :goal (explored pt_obs_12245_-8578))
        (move-0 explore-1 (at-r ?r pt_aav_12245_-8578))
        (explore-2 :goal (explored pt_obs_14242_-8578))
        (move-1 explore-2 (at-r ?r pt_aav_14242_-8578))
        (explore-3 :goal (explored pt_obs_14242_-6582))
        (move-2 explore-3 (at-r ?r pt_aav_14242_-6582))
        (explore-4 :goal (explored pt_obs_14242_-4585))
        (move-3 explore-4 (at-r ?r pt_aav_14242_-4585))
        (explore-5 :goal (explored pt_obs_14242_-2588))
        (move-4 explore-5 (at-r ?r pt_aav_14242_-2588))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)
        (explore-2 move-2)
        (explore-3 move-3)
        (explore-4 move-4)

  )
)


(:action patrolaav__pt_aav_16239_-6582__pt_aav_18235_-6582
  :parameters (?r - aav)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_aav_16239_-6582))
  :effect (and (explored pt_obs_18235_-6582) (explored pt_obs_16239_-6582) (explored pt_obs_16239_-8578) (at-r ?r pt_aav_18235_-6582))
  
  :methods(

      :method direct
      :actions (move-0 (move-aav ?r pt_aav_16239_-6582 pt_aav_16239_-8578))
        (move-1 (move-aav ?r pt_aav_16239_-8578 pt_aav_18235_-6582))
        (explore-0 (observe-aav ?r pt_aav_16239_-6582 pt_obs_16239_-6582))
        (explore-1 (observe-aav ?r pt_aav_16239_-8578 pt_obs_16239_-8578))
        (explore-2 (observe-aav ?r pt_aav_18235_-6582 pt_obs_18235_-6582))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_aav_16239_-6582))
        (move-0 move-1 (at-r ?r pt_aav_16239_-8578))
        (move-1 :goal (at-r ?r pt_aav_18235_-6582))
        (explore-0 :goal (explored pt_obs_16239_-6582))
        (:init explore-0 (at-r ?r pt_aav_16239_-6582))
        (explore-1 :goal (explored pt_obs_16239_-8578))
        (move-0 explore-1 (at-r ?r pt_aav_16239_-8578))
        (explore-2 :goal (explored pt_obs_18235_-6582))
        (move-1 explore-2 (at-r ?r pt_aav_18235_-6582))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)

  )
)


(:action patrolaav__pt_aav_18235_-6582__pt_aav_16239_-6582
  :parameters (?r - aav)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_aav_18235_-6582))
  :effect (and (explored pt_obs_18235_-6582) (explored pt_obs_16239_-6582) (explored pt_obs_16239_-8578) (at-r ?r pt_aav_16239_-6582))
  
  :methods(

      :method direct
      :actions (move-0 (move-aav ?r pt_aav_18235_-6582 pt_aav_16239_-8578))
        (move-1 (move-aav ?r pt_aav_16239_-8578 pt_aav_16239_-6582))
        (explore-0 (observe-aav ?r pt_aav_18235_-6582 pt_obs_18235_-6582))
        (explore-1 (observe-aav ?r pt_aav_16239_-8578 pt_obs_16239_-8578))
        (explore-2 (observe-aav ?r pt_aav_16239_-6582 pt_obs_16239_-6582))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_aav_18235_-6582))
        (move-0 move-1 (at-r ?r pt_aav_16239_-8578))
        (move-1 :goal (at-r ?r pt_aav_16239_-6582))
        (explore-0 :goal (explored pt_obs_18235_-6582))
        (:init explore-0 (at-r ?r pt_aav_18235_-6582))
        (explore-1 :goal (explored pt_obs_16239_-8578))
        (move-0 explore-1 (at-r ?r pt_aav_16239_-8578))
        (explore-2 :goal (explored pt_obs_16239_-6582))
        (move-1 explore-2 (at-r ?r pt_aav_16239_-6582))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)

  )
)


(:action patrolaav__pt_aav_18235_-592__pt_aav_16239_-2588
  :parameters (?r - aav)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_aav_18235_-592))
  :effect (and (explored pt_obs_18235_-4585) (explored pt_obs_16239_-4585) (explored pt_obs_18235_-2588) (explored pt_obs_18235_-592) (explored pt_obs_16239_-2588) (at-r ?r pt_aav_16239_-2588))
  
  :methods(

      :method direct
      :actions (move-0 (move-aav ?r pt_aav_18235_-592 pt_aav_18235_-2588))
        (move-1 (move-aav ?r pt_aav_18235_-2588 pt_aav_18235_-4585))
        (move-2 (move-aav ?r pt_aav_18235_-4585 pt_aav_16239_-4585))
        (move-3 (move-aav ?r pt_aav_16239_-4585 pt_aav_16239_-2588))
        (explore-0 (observe-aav ?r pt_aav_18235_-592 pt_obs_18235_-592))
        (explore-1 (observe-aav ?r pt_aav_18235_-2588 pt_obs_18235_-2588))
        (explore-2 (observe-aav ?r pt_aav_18235_-4585 pt_obs_18235_-4585))
        (explore-3 (observe-aav ?r pt_aav_16239_-4585 pt_obs_16239_-4585))
        (explore-4 (observe-aav ?r pt_aav_16239_-2588 pt_obs_16239_-2588))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_aav_18235_-592))
        (move-0 move-1 (at-r ?r pt_aav_18235_-2588))
        (move-1 move-2 (at-r ?r pt_aav_18235_-4585))
        (move-2 move-3 (at-r ?r pt_aav_16239_-4585))
        (move-3 :goal (at-r ?r pt_aav_16239_-2588))
        (explore-0 :goal (explored pt_obs_18235_-592))
        (:init explore-0 (at-r ?r pt_aav_18235_-592))
        (explore-1 :goal (explored pt_obs_18235_-2588))
        (move-0 explore-1 (at-r ?r pt_aav_18235_-2588))
        (explore-2 :goal (explored pt_obs_18235_-4585))
        (move-1 explore-2 (at-r ?r pt_aav_18235_-4585))
        (explore-3 :goal (explored pt_obs_16239_-4585))
        (move-2 explore-3 (at-r ?r pt_aav_16239_-4585))
        (explore-4 :goal (explored pt_obs_16239_-2588))
        (move-3 explore-4 (at-r ?r pt_aav_16239_-2588))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)
        (explore-2 move-2)
        (explore-3 move-3)

  )
)


(:action patrolaav__pt_aav_16239_-2588__pt_aav_18235_-592
  :parameters (?r - aav)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_aav_16239_-2588))
  :effect (and (explored pt_obs_18235_-4585) (explored pt_obs_16239_-4585) (explored pt_obs_18235_-2588) (explored pt_obs_18235_-592) (explored pt_obs_16239_-2588) (at-r ?r pt_aav_18235_-592))
  
  :methods(

      :method direct
      :actions (move-0 (move-aav ?r pt_aav_16239_-2588 pt_aav_16239_-4585))
        (move-1 (move-aav ?r pt_aav_16239_-4585 pt_aav_18235_-4585))
        (move-2 (move-aav ?r pt_aav_18235_-4585 pt_aav_18235_-2588))
        (move-3 (move-aav ?r pt_aav_18235_-2588 pt_aav_18235_-592))
        (explore-0 (observe-aav ?r pt_aav_16239_-2588 pt_obs_16239_-2588))
        (explore-1 (observe-aav ?r pt_aav_16239_-4585 pt_obs_16239_-4585))
        (explore-2 (observe-aav ?r pt_aav_18235_-4585 pt_obs_18235_-4585))
        (explore-3 (observe-aav ?r pt_aav_18235_-2588 pt_obs_18235_-2588))
        (explore-4 (observe-aav ?r pt_aav_18235_-592 pt_obs_18235_-592))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_aav_16239_-2588))
        (move-0 move-1 (at-r ?r pt_aav_16239_-4585))
        (move-1 move-2 (at-r ?r pt_aav_18235_-4585))
        (move-2 move-3 (at-r ?r pt_aav_18235_-2588))
        (move-3 :goal (at-r ?r pt_aav_18235_-592))
        (explore-0 :goal (explored pt_obs_16239_-2588))
        (:init explore-0 (at-r ?r pt_aav_16239_-2588))
        (explore-1 :goal (explored pt_obs_16239_-4585))
        (move-0 explore-1 (at-r ?r pt_aav_16239_-4585))
        (explore-2 :goal (explored pt_obs_18235_-4585))
        (move-1 explore-2 (at-r ?r pt_aav_18235_-4585))
        (explore-3 :goal (explored pt_obs_18235_-2588))
        (move-2 explore-3 (at-r ?r pt_aav_18235_-2588))
        (explore-4 :goal (explored pt_obs_18235_-592))
        (move-3 explore-4 (at-r ?r pt_aav_18235_-592))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)
        (explore-2 move-2)
        (explore-3 move-3)

  )
)


(:action patrolaav__pt_aav_22229_-2588__pt_aav_22229_-2588
  :parameters (?r - aav)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_aav_22229_-2588))
  :effect (and (explored pt_obs_22229_-2588) (at-r ?r pt_aav_22229_-2588))
  
  :methods(

      :method direct
      :actions (explore-0 (observe-aav ?r pt_aav_22229_-2588 pt_obs_22229_-2588))
      
      :precondition
      :causal-links (:init :goal (at-r ?r pt_aav_22229_-2588))
        (explore-0 :goal (explored pt_obs_22229_-2588))
        (:init explore-0 (at-r ?r pt_aav_22229_-2588))
      :temporal-links 

  )
)


(:action patrolaav__pt_aav_18235_1404__pt_aav_16239_-592
  :parameters (?r - aav)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_aav_18235_1404))
  :effect (and (explored pt_obs_16239_3401) (explored pt_obs_18235_1404) (explored pt_obs_16239_1404) (explored pt_obs_16239_-592) (at-r ?r pt_aav_16239_-592))
  
  :methods(

      :method direct
      :actions (move-0 (move-aav ?r pt_aav_18235_1404 pt_aav_16239_3401))
        (move-1 (move-aav ?r pt_aav_16239_3401 pt_aav_16239_1404))
        (move-2 (move-aav ?r pt_aav_16239_1404 pt_aav_16239_-592))
        (explore-0 (observe-aav ?r pt_aav_18235_1404 pt_obs_18235_1404))
        (explore-1 (observe-aav ?r pt_aav_16239_3401 pt_obs_16239_3401))
        (explore-2 (observe-aav ?r pt_aav_16239_1404 pt_obs_16239_1404))
        (explore-3 (observe-aav ?r pt_aav_16239_-592 pt_obs_16239_-592))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_aav_18235_1404))
        (move-0 move-1 (at-r ?r pt_aav_16239_3401))
        (move-1 move-2 (at-r ?r pt_aav_16239_1404))
        (move-2 :goal (at-r ?r pt_aav_16239_-592))
        (explore-0 :goal (explored pt_obs_18235_1404))
        (:init explore-0 (at-r ?r pt_aav_18235_1404))
        (explore-1 :goal (explored pt_obs_16239_3401))
        (move-0 explore-1 (at-r ?r pt_aav_16239_3401))
        (explore-2 :goal (explored pt_obs_16239_1404))
        (move-1 explore-2 (at-r ?r pt_aav_16239_1404))
        (explore-3 :goal (explored pt_obs_16239_-592))
        (move-2 explore-3 (at-r ?r pt_aav_16239_-592))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)
        (explore-2 move-2)

  )
)


(:action patrolaav__pt_aav_16239_-592__pt_aav_18235_1404
  :parameters (?r - aav)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_aav_16239_-592))
  :effect (and (explored pt_obs_16239_-592) (explored pt_obs_18235_1404) (explored pt_obs_16239_1404) (explored pt_obs_16239_3401) (at-r ?r pt_aav_18235_1404))
  
  :methods(

      :method direct
      :actions (move-0 (move-aav ?r pt_aav_16239_-592 pt_aav_16239_1404))
        (move-1 (move-aav ?r pt_aav_16239_1404 pt_aav_16239_3401))
        (move-2 (move-aav ?r pt_aav_16239_3401 pt_aav_18235_1404))
        (explore-0 (observe-aav ?r pt_aav_16239_-592 pt_obs_16239_-592))
        (explore-1 (observe-aav ?r pt_aav_16239_1404 pt_obs_16239_1404))
        (explore-2 (observe-aav ?r pt_aav_16239_3401 pt_obs_16239_3401))
        (explore-3 (observe-aav ?r pt_aav_18235_1404 pt_obs_18235_1404))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_aav_16239_-592))
        (move-0 move-1 (at-r ?r pt_aav_16239_1404))
        (move-1 move-2 (at-r ?r pt_aav_16239_3401))
        (move-2 :goal (at-r ?r pt_aav_18235_1404))
        (explore-0 :goal (explored pt_obs_16239_-592))
        (:init explore-0 (at-r ?r pt_aav_16239_-592))
        (explore-1 :goal (explored pt_obs_16239_1404))
        (move-0 explore-1 (at-r ?r pt_aav_16239_1404))
        (explore-2 :goal (explored pt_obs_16239_3401))
        (move-1 explore-2 (at-r ?r pt_aav_16239_3401))
        (explore-3 :goal (explored pt_obs_18235_1404))
        (move-2 explore-3 (at-r ?r pt_aav_18235_1404))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)
        (explore-2 move-2)

  )
)


(:action patrolaav__pt_aav_10249_-4585__pt_aav_12245_-2588
  :parameters (?r - aav)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_aav_10249_-4585))
  :effect (and (explored pt_obs_12245_-2588) (explored pt_obs_10249_-4585) (at-r ?r pt_aav_12245_-2588))
  
  :methods(

      :method direct
      :actions (move-0 (move-aav ?r pt_aav_10249_-4585 pt_aav_12245_-2588))
        (explore-0 (observe-aav ?r pt_aav_10249_-4585 pt_obs_10249_-4585))
        (explore-1 (observe-aav ?r pt_aav_12245_-2588 pt_obs_12245_-2588))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_aav_10249_-4585))
        (move-0 :goal (at-r ?r pt_aav_12245_-2588))
        (explore-0 :goal (explored pt_obs_10249_-4585))
        (:init explore-0 (at-r ?r pt_aav_10249_-4585))
        (explore-1 :goal (explored pt_obs_12245_-2588))
        (move-0 explore-1 (at-r ?r pt_aav_12245_-2588))
      :temporal-links (explore-0 move-0)

  )
)


(:action patrolaav__pt_aav_12245_-2588__pt_aav_10249_-4585
  :parameters (?r - aav)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_aav_12245_-2588))
  :effect (and (explored pt_obs_12245_-2588) (explored pt_obs_10249_-4585) (at-r ?r pt_aav_10249_-4585))
  
  :methods(

      :method direct
      :actions (move-0 (move-aav ?r pt_aav_12245_-2588 pt_aav_10249_-4585))
        (explore-0 (observe-aav ?r pt_aav_12245_-2588 pt_obs_12245_-2588))
        (explore-1 (observe-aav ?r pt_aav_10249_-4585 pt_obs_10249_-4585))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_aav_12245_-2588))
        (move-0 :goal (at-r ?r pt_aav_10249_-4585))
        (explore-0 :goal (explored pt_obs_12245_-2588))
        (:init explore-0 (at-r ?r pt_aav_12245_-2588))
        (explore-1 :goal (explored pt_obs_10249_-4585))
        (move-0 explore-1 (at-r ?r pt_aav_10249_-4585))
      :temporal-links (explore-0 move-0)

  )
)


(:action patrolaav__pt_aav_14242_3401__pt_aav_14242_1404
  :parameters (?r - aav)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_aav_14242_3401))
  :effect (and (explored pt_obs_14242_3401) (explored pt_obs_14242_1404) (explored pt_obs_10249_5397) (explored pt_obs_12245_3401) (explored pt_obs_12245_5397) (at-r ?r pt_aav_14242_1404))
  
  :methods(

      :method direct
      :actions (move-0 (move-aav ?r pt_aav_14242_3401 pt_aav_12245_5397))
        (move-1 (move-aav ?r pt_aav_12245_5397 pt_aav_10249_5397))
        (move-2 (move-aav ?r pt_aav_10249_5397 pt_aav_12245_3401))
        (move-3 (move-aav ?r pt_aav_12245_3401 pt_aav_14242_1404))
        (explore-0 (observe-aav ?r pt_aav_14242_3401 pt_obs_14242_3401))
        (explore-1 (observe-aav ?r pt_aav_12245_5397 pt_obs_12245_5397))
        (explore-2 (observe-aav ?r pt_aav_10249_5397 pt_obs_10249_5397))
        (explore-3 (observe-aav ?r pt_aav_12245_3401 pt_obs_12245_3401))
        (explore-4 (observe-aav ?r pt_aav_14242_1404 pt_obs_14242_1404))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_aav_14242_3401))
        (move-0 move-1 (at-r ?r pt_aav_12245_5397))
        (move-1 move-2 (at-r ?r pt_aav_10249_5397))
        (move-2 move-3 (at-r ?r pt_aav_12245_3401))
        (move-3 :goal (at-r ?r pt_aav_14242_1404))
        (explore-0 :goal (explored pt_obs_14242_3401))
        (:init explore-0 (at-r ?r pt_aav_14242_3401))
        (explore-1 :goal (explored pt_obs_12245_5397))
        (move-0 explore-1 (at-r ?r pt_aav_12245_5397))
        (explore-2 :goal (explored pt_obs_10249_5397))
        (move-1 explore-2 (at-r ?r pt_aav_10249_5397))
        (explore-3 :goal (explored pt_obs_12245_3401))
        (move-2 explore-3 (at-r ?r pt_aav_12245_3401))
        (explore-4 :goal (explored pt_obs_14242_1404))
        (move-3 explore-4 (at-r ?r pt_aav_14242_1404))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)
        (explore-2 move-2)
        (explore-3 move-3)

  )
)


(:action patrolaav__pt_aav_14242_1404__pt_aav_14242_3401
  :parameters (?r - aav)
  :conflict-with (at-r ?r *)
  
  :precondition (and (at-r ?r pt_aav_14242_1404))
  :effect (and (explored pt_obs_14242_1404) (explored pt_obs_14242_3401) (explored pt_obs_10249_5397) (explored pt_obs_12245_3401) (explored pt_obs_12245_5397) (at-r ?r pt_aav_14242_3401))
  
  :methods(

      :method direct
      :actions (move-0 (move-aav ?r pt_aav_14242_1404 pt_aav_12245_3401))
        (move-1 (move-aav ?r pt_aav_12245_3401 pt_aav_10249_5397))
        (move-2 (move-aav ?r pt_aav_10249_5397 pt_aav_12245_5397))
        (move-3 (move-aav ?r pt_aav_12245_5397 pt_aav_14242_3401))
        (explore-0 (observe-aav ?r pt_aav_14242_1404 pt_obs_14242_1404))
        (explore-1 (observe-aav ?r pt_aav_12245_3401 pt_obs_12245_3401))
        (explore-2 (observe-aav ?r pt_aav_10249_5397 pt_obs_10249_5397))
        (explore-3 (observe-aav ?r pt_aav_12245_5397 pt_obs_12245_5397))
        (explore-4 (observe-aav ?r pt_aav_14242_3401 pt_obs_14242_3401))
      
      :precondition
      :causal-links (:init move-0 (at-r ?r pt_aav_14242_1404))
        (move-0 move-1 (at-r ?r pt_aav_12245_3401))
        (move-1 move-2 (at-r ?r pt_aav_10249_5397))
        (move-2 move-3 (at-r ?r pt_aav_12245_5397))
        (move-3 :goal (at-r ?r pt_aav_14242_3401))
        (explore-0 :goal (explored pt_obs_14242_1404))
        (:init explore-0 (at-r ?r pt_aav_14242_1404))
        (explore-1 :goal (explored pt_obs_12245_3401))
        (move-0 explore-1 (at-r ?r pt_aav_12245_3401))
        (explore-2 :goal (explored pt_obs_10249_5397))
        (move-1 explore-2 (at-r ?r pt_aav_10249_5397))
        (explore-3 :goal (explored pt_obs_12245_5397))
        (move-2 explore-3 (at-r ?r pt_aav_12245_5397))
        (explore-4 :goal (explored pt_obs_14242_3401))
        (move-3 explore-4 (at-r ?r pt_aav_14242_3401))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)
        (explore-2 move-2)
        (explore-3 move-3)

  )
)

)
