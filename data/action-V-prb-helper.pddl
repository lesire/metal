
(define (domain-helper action)
    (:options :abstractOnly :erasePlansWhenAbstractMet)
    (:allowed-actions move communicate has-communicated)
    (:low-priority-predicates )

    
(:action patrol_ressac2_1__ressac2pt_22229_-592_0__ressac2pt_19164_-6165_0
  :parameters ()
  :conflict-with (at-r ressac2 *)
  
  :precondition (and (at-r ressac2 ressac2pt_22229_-592_0))
  :effect (and (explored ptobs_16400_-2630) (explored ptobs_18495_-1244) (explored ptobs_19229_-6710) (at-r ressac2 ressac2pt_19164_-6165_0))
  
  :methods(

      :method patrol_ressac2_1__ressac2pt_22229_-592_0__ressac2pt_19164_-6165_0_d_1
      :actions (move-0 (move ressac2 ressac2pt_22229_-592_0 ressac2pt_21379_1030_0))
        (move-1 (move ressac2 ressac2pt_21379_1030_0 ressac2pt_18195_-1119_0))
        (move-2 (move ressac2 ressac2pt_18195_-1119_0 ressac2pt_16239_-2588_0))
        (move-3 (move ressac2 ressac2pt_16239_-2588_0 ressac2pt_19164_-6165_0))
        (explore-0 (observe ressac2 ressac2pt_18195_-1119_0 ptobs_18495_-1244))
        (explore-1 (observe ressac2 ressac2pt_16239_-2588_0 ptobs_16400_-2630))
        (explore-2 (observe ressac2 ressac2pt_19164_-6165_0 ptobs_19229_-6710))
      
      :precondition
      :causal-links (:init move-0 (at-r ressac2 ressac2pt_22229_-592_0))
        (move-0 move-1 (at-r ressac2 ressac2pt_21379_1030_0))
        (move-1 move-2 (at-r ressac2 ressac2pt_18195_-1119_0))
        (move-2 move-3 (at-r ressac2 ressac2pt_16239_-2588_0))
        (move-3 :goal (at-r ressac2 ressac2pt_19164_-6165_0))
        (explore-0 :goal (explored ptobs_18495_-1244))
        (move-1 explore-0 (at-r ressac2 ressac2pt_18195_-1119_0))
        (explore-1 :goal (explored ptobs_16400_-2630))
        (move-2 explore-1 (at-r ressac2 ressac2pt_16239_-2588_0))
        (explore-2 :goal (explored ptobs_19229_-6710))
        (move-3 explore-2 (at-r ressac2 ressac2pt_19164_-6165_0))
      :temporal-links (explore-0 move-2)
        (explore-1 move-3)

  )
)


(:action patrol_ressac2_1__ressac2pt_19164_-6165_0__ressac2pt_22229_-592_0
  :parameters ()
  :conflict-with (at-r ressac2 *)
  
  :precondition (and (at-r ressac2 ressac2pt_19164_-6165_0))
  :effect (and (explored ptobs_16400_-2630) (explored ptobs_18495_-1244) (explored ptobs_19229_-6710) (at-r ressac2 ressac2pt_22229_-592_0))
  
  :methods(

      :method patrol_ressac2_1__ressac2pt_19164_-6165_0__ressac2pt_22229_-592_0_i_1
      :actions (move-0 (move ressac2 ressac2pt_19164_-6165_0 ressac2pt_16239_-2588_0))
        (move-1 (move ressac2 ressac2pt_16239_-2588_0 ressac2pt_18195_-1119_0))
        (move-2 (move ressac2 ressac2pt_18195_-1119_0 ressac2pt_21379_1030_0))
        (move-3 (move ressac2 ressac2pt_21379_1030_0 ressac2pt_22229_-592_0))
        (explore-0 (observe ressac2 ressac2pt_19164_-6165_0 ptobs_19229_-6710))
        (explore-1 (observe ressac2 ressac2pt_16239_-2588_0 ptobs_16400_-2630))
        (explore-2 (observe ressac2 ressac2pt_18195_-1119_0 ptobs_18495_-1244))
      
      :precondition
      :causal-links (:init move-0 (at-r ressac2 ressac2pt_19164_-6165_0))
        (move-0 move-1 (at-r ressac2 ressac2pt_16239_-2588_0))
        (move-1 move-2 (at-r ressac2 ressac2pt_18195_-1119_0))
        (move-2 move-3 (at-r ressac2 ressac2pt_21379_1030_0))
        (move-3 :goal (at-r ressac2 ressac2pt_22229_-592_0))
        (explore-0 :goal (explored ptobs_19229_-6710))
        (:init explore-0 (at-r ressac2 ressac2pt_19164_-6165_0))
        (explore-1 :goal (explored ptobs_16400_-2630))
        (move-0 explore-1 (at-r ressac2 ressac2pt_16239_-2588_0))
        (explore-2 :goal (explored ptobs_18495_-1244))
        (move-1 explore-2 (at-r ressac2 ressac2pt_18195_-1119_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)
        (explore-2 move-2)

  )
)


(:action patrol_ressac2_2__ressac2pt_19164_-6165_0__ressac2pt_19500_269_0
  :parameters ()
  :conflict-with (at-r ressac2 *)
  
  :precondition (and (at-r ressac2 ressac2pt_19164_-6165_0))
  :effect (and (explored ptobs_15750_-8600) (explored ptobs_14184_-4615) (explored ptobs_19814_225) (explored ptobs_22884_-4144) (explored ptobs_19229_-6710) (at-r ressac2 ressac2pt_19500_269_0))
  
  :methods(

      :method patrol_ressac2_2__ressac2pt_19164_-6165_0__ressac2pt_19500_269_0_d_2
      :actions (move-0 (move ressac2 ressac2pt_19164_-6165_0 ressac2pt_15425_-8600_0))
        (move-1 (move ressac2 ressac2pt_15425_-8600_0 ressac2pt_14242_-4585_0))
        (move-2 (move ressac2 ressac2pt_14242_-4585_0 ressac2pt_18235_-2588_0))
        (move-3 (move ressac2 ressac2pt_18235_-2588_0 ressac2pt_20232_-4585_0))
        (move-4 (move ressac2 ressac2pt_20232_-4585_0 ressac2pt_22759_-3960_0))
        (move-5 (move ressac2 ressac2pt_22759_-3960_0 ressac2pt_19500_269_0))
        (explore-0 (observe ressac2 ressac2pt_19164_-6165_0 ptobs_19229_-6710))
        (explore-1 (observe ressac2 ressac2pt_15425_-8600_0 ptobs_15750_-8600))
        (explore-2 (observe ressac2 ressac2pt_14242_-4585_0 ptobs_14184_-4615))
        (explore-3 (observe ressac2 ressac2pt_22759_-3960_0 ptobs_22884_-4144))
        (explore-4 (observe ressac2 ressac2pt_19500_269_0 ptobs_19814_225))
      
      :precondition
      :causal-links (:init move-0 (at-r ressac2 ressac2pt_19164_-6165_0))
        (move-0 move-1 (at-r ressac2 ressac2pt_15425_-8600_0))
        (move-1 move-2 (at-r ressac2 ressac2pt_14242_-4585_0))
        (move-2 move-3 (at-r ressac2 ressac2pt_18235_-2588_0))
        (move-3 move-4 (at-r ressac2 ressac2pt_20232_-4585_0))
        (move-4 move-5 (at-r ressac2 ressac2pt_22759_-3960_0))
        (move-5 :goal (at-r ressac2 ressac2pt_19500_269_0))
        (explore-0 :goal (explored ptobs_19229_-6710))
        (:init explore-0 (at-r ressac2 ressac2pt_19164_-6165_0))
        (explore-1 :goal (explored ptobs_15750_-8600))
        (move-0 explore-1 (at-r ressac2 ressac2pt_15425_-8600_0))
        (explore-2 :goal (explored ptobs_14184_-4615))
        (move-1 explore-2 (at-r ressac2 ressac2pt_14242_-4585_0))
        (explore-3 :goal (explored ptobs_22884_-4144))
        (move-4 explore-3 (at-r ressac2 ressac2pt_22759_-3960_0))
        (explore-4 :goal (explored ptobs_19814_225))
        (move-5 explore-4 (at-r ressac2 ressac2pt_19500_269_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)
        (explore-2 move-2)
        (explore-3 move-5)

  )
)


(:action patrol_ressac2_2__ressac2pt_19500_269_0__ressac2pt_19164_-6165_0
  :parameters ()
  :conflict-with (at-r ressac2 *)
  
  :precondition (and (at-r ressac2 ressac2pt_19500_269_0))
  :effect (and (explored ptobs_15750_-8600) (explored ptobs_14184_-4615) (explored ptobs_19814_225) (explored ptobs_19229_-6710) (explored ptobs_22884_-4144) (at-r ressac2 ressac2pt_19164_-6165_0))
  
  :methods(

      :method patrol_ressac2_2__ressac2pt_19500_269_0__ressac2pt_19164_-6165_0_i_2
      :actions (move-0 (move ressac2 ressac2pt_19500_269_0 ressac2pt_22759_-3960_0))
        (move-1 (move ressac2 ressac2pt_22759_-3960_0 ressac2pt_20232_-4585_0))
        (move-2 (move ressac2 ressac2pt_20232_-4585_0 ressac2pt_18235_-2588_0))
        (move-3 (move ressac2 ressac2pt_18235_-2588_0 ressac2pt_14242_-4585_0))
        (move-4 (move ressac2 ressac2pt_14242_-4585_0 ressac2pt_15425_-8600_0))
        (move-5 (move ressac2 ressac2pt_15425_-8600_0 ressac2pt_19164_-6165_0))
        (explore-0 (observe ressac2 ressac2pt_19500_269_0 ptobs_19814_225))
        (explore-1 (observe ressac2 ressac2pt_22759_-3960_0 ptobs_22884_-4144))
        (explore-2 (observe ressac2 ressac2pt_14242_-4585_0 ptobs_14184_-4615))
        (explore-3 (observe ressac2 ressac2pt_15425_-8600_0 ptobs_15750_-8600))
        (explore-4 (observe ressac2 ressac2pt_19164_-6165_0 ptobs_19229_-6710))
      
      :precondition
      :causal-links (:init move-0 (at-r ressac2 ressac2pt_19500_269_0))
        (move-0 move-1 (at-r ressac2 ressac2pt_22759_-3960_0))
        (move-1 move-2 (at-r ressac2 ressac2pt_20232_-4585_0))
        (move-2 move-3 (at-r ressac2 ressac2pt_18235_-2588_0))
        (move-3 move-4 (at-r ressac2 ressac2pt_14242_-4585_0))
        (move-4 move-5 (at-r ressac2 ressac2pt_15425_-8600_0))
        (move-5 :goal (at-r ressac2 ressac2pt_19164_-6165_0))
        (explore-0 :goal (explored ptobs_19814_225))
        (:init explore-0 (at-r ressac2 ressac2pt_19500_269_0))
        (explore-1 :goal (explored ptobs_22884_-4144))
        (move-0 explore-1 (at-r ressac2 ressac2pt_22759_-3960_0))
        (explore-2 :goal (explored ptobs_14184_-4615))
        (move-3 explore-2 (at-r ressac2 ressac2pt_14242_-4585_0))
        (explore-3 :goal (explored ptobs_15750_-8600))
        (move-4 explore-3 (at-r ressac2 ressac2pt_15425_-8600_0))
        (explore-4 :goal (explored ptobs_19229_-6710))
        (move-5 explore-4 (at-r ressac2 ressac2pt_19164_-6165_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)
        (explore-2 move-4)
        (explore-3 move-5)

  )
)


(:action patrol_mana_1__agvpt_20942_2602_0__agvpt_12305_2615_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agvpt_20942_2602_0))
  :effect (and (explored ptobs_19464_4275) (explored ptobs_12070_2605) (explored ptobs_14889_4915) (at-r mana agvpt_12305_2615_0))
  
  :methods(

      :method patrol_mana_1__agvpt_20942_2602_0__agvpt_12305_2615_0_d_1
      :actions (move-0 (move mana agvpt_20942_2602_0 agvpt_19478_4554_0))
        (move-1 (move mana agvpt_19478_4554_0 agvpt_18502_5530_0))
        (move-2 (move mana agvpt_18502_5530_0 agvpt_17037_6506_0))
        (move-3 (move mana agvpt_17037_6506_0 agvpt_15429_5345_0))
        (move-4 (move mana agvpt_15429_5345_0 agvpt_14109_4066_0))
        (move-5 (move mana agvpt_14109_4066_0 agvpt_12305_2615_0))
        (explore-0 (observe mana agvpt_19478_4554_0 ptobs_19464_4275))
        (explore-1 (observe mana agvpt_15429_5345_0 ptobs_14889_4915))
        (explore-2 (observe mana agvpt_12305_2615_0 ptobs_12070_2605))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agvpt_20942_2602_0))
        (move-0 move-1 (at-r mana agvpt_19478_4554_0))
        (move-1 move-2 (at-r mana agvpt_18502_5530_0))
        (move-2 move-3 (at-r mana agvpt_17037_6506_0))
        (move-3 move-4 (at-r mana agvpt_15429_5345_0))
        (move-4 move-5 (at-r mana agvpt_14109_4066_0))
        (move-5 :goal (at-r mana agvpt_12305_2615_0))
        (explore-0 :goal (explored ptobs_19464_4275))
        (move-0 explore-0 (at-r mana agvpt_19478_4554_0))
        (explore-1 :goal (explored ptobs_14889_4915))
        (move-3 explore-1 (at-r mana agvpt_15429_5345_0))
        (explore-2 :goal (explored ptobs_12070_2605))
        (move-5 explore-2 (at-r mana agvpt_12305_2615_0))
      :temporal-links (explore-0 move-1)
        (explore-1 move-4)

  )
)


(:action patrol_mana_1__agvpt_12305_2615_0__agvpt_20942_2602_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agvpt_12305_2615_0))
  :effect (and (explored ptobs_19464_4275) (explored ptobs_12070_2605) (explored ptobs_14889_4915) (at-r mana agvpt_20942_2602_0))
  
  :methods(

      :method patrol_mana_1__agvpt_12305_2615_0__agvpt_20942_2602_0_i_1
      :actions (move-0 (move mana agvpt_12305_2615_0 agvpt_14109_4066_0))
        (move-1 (move mana agvpt_14109_4066_0 agvpt_15429_5345_0))
        (move-2 (move mana agvpt_15429_5345_0 agvpt_17037_6506_0))
        (move-3 (move mana agvpt_17037_6506_0 agvpt_18502_5530_0))
        (move-4 (move mana agvpt_18502_5530_0 agvpt_19478_4554_0))
        (move-5 (move mana agvpt_19478_4554_0 agvpt_20942_2602_0))
        (explore-0 (observe mana agvpt_12305_2615_0 ptobs_12070_2605))
        (explore-1 (observe mana agvpt_15429_5345_0 ptobs_14889_4915))
        (explore-2 (observe mana agvpt_19478_4554_0 ptobs_19464_4275))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agvpt_12305_2615_0))
        (move-0 move-1 (at-r mana agvpt_14109_4066_0))
        (move-1 move-2 (at-r mana agvpt_15429_5345_0))
        (move-2 move-3 (at-r mana agvpt_17037_6506_0))
        (move-3 move-4 (at-r mana agvpt_18502_5530_0))
        (move-4 move-5 (at-r mana agvpt_19478_4554_0))
        (move-5 :goal (at-r mana agvpt_20942_2602_0))
        (explore-0 :goal (explored ptobs_12070_2605))
        (:init explore-0 (at-r mana agvpt_12305_2615_0))
        (explore-1 :goal (explored ptobs_14889_4915))
        (move-1 explore-1 (at-r mana agvpt_15429_5345_0))
        (explore-2 :goal (explored ptobs_19464_4275))
        (move-4 explore-2 (at-r mana agvpt_19478_4554_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-2)
        (explore-2 move-5)

  )
)


(:action patrol_mana_4__agvpt_19270_-6544_0__agvpt_12245_-4889_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agvpt_19270_-6544_0))
  :effect (and (explored ptobs_12420_-5014) (explored ptobs_15645_-10244) (explored ptobs_19229_-6710) (at-r mana agvpt_12245_-4889_0))
  
  :methods(

      :method patrol_mana_4__agvpt_19270_-6544_0__agvpt_12245_-4889_0_d_4
      :actions (move-0 (move mana agvpt_19270_-6544_0 agvpt_17525_-8623_0))
        (move-1 (move mana agvpt_17525_-8623_0 agvpt_16061_-10087_0))
        (move-2 (move mana agvpt_16061_-10087_0 agvpt_13539_-6550_0))
        (move-3 (move mana agvpt_13539_-6550_0 agvpt_12245_-4889_0))
        (explore-0 (observe mana agvpt_19270_-6544_0 ptobs_19229_-6710))
        (explore-1 (observe mana agvpt_16061_-10087_0 ptobs_15645_-10244))
        (explore-2 (observe mana agvpt_12245_-4889_0 ptobs_12420_-5014))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agvpt_19270_-6544_0))
        (move-0 move-1 (at-r mana agvpt_17525_-8623_0))
        (move-1 move-2 (at-r mana agvpt_16061_-10087_0))
        (move-2 move-3 (at-r mana agvpt_13539_-6550_0))
        (move-3 :goal (at-r mana agvpt_12245_-4889_0))
        (explore-0 :goal (explored ptobs_19229_-6710))
        (:init explore-0 (at-r mana agvpt_19270_-6544_0))
        (explore-1 :goal (explored ptobs_15645_-10244))
        (move-1 explore-1 (at-r mana agvpt_16061_-10087_0))
        (explore-2 :goal (explored ptobs_12420_-5014))
        (move-3 explore-2 (at-r mana agvpt_12245_-4889_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-2)

  )
)


(:action patrol_mana_4__agvpt_12245_-4889_0__agvpt_19270_-6544_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agvpt_12245_-4889_0))
  :effect (and (explored ptobs_12420_-5014) (explored ptobs_15645_-10244) (explored ptobs_19229_-6710) (at-r mana agvpt_19270_-6544_0))
  
  :methods(

      :method patrol_mana_4__agvpt_12245_-4889_0__agvpt_19270_-6544_0_i_4
      :actions (move-0 (move mana agvpt_12245_-4889_0 agvpt_13539_-6550_0))
        (move-1 (move mana agvpt_13539_-6550_0 agvpt_16061_-10087_0))
        (move-2 (move mana agvpt_16061_-10087_0 agvpt_17525_-8623_0))
        (move-3 (move mana agvpt_17525_-8623_0 agvpt_19270_-6544_0))
        (explore-0 (observe mana agvpt_12245_-4889_0 ptobs_12420_-5014))
        (explore-1 (observe mana agvpt_16061_-10087_0 ptobs_15645_-10244))
        (explore-2 (observe mana agvpt_19270_-6544_0 ptobs_19229_-6710))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agvpt_12245_-4889_0))
        (move-0 move-1 (at-r mana agvpt_13539_-6550_0))
        (move-1 move-2 (at-r mana agvpt_16061_-10087_0))
        (move-2 move-3 (at-r mana agvpt_17525_-8623_0))
        (move-3 :goal (at-r mana agvpt_19270_-6544_0))
        (explore-0 :goal (explored ptobs_12420_-5014))
        (:init explore-0 (at-r mana agvpt_12245_-4889_0))
        (explore-1 :goal (explored ptobs_15645_-10244))
        (move-1 explore-1 (at-r mana agvpt_16061_-10087_0))
        (explore-2 :goal (explored ptobs_19229_-6710))
        (move-3 explore-2 (at-r mana agvpt_19270_-6544_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-2)

  )
)


(:action patrol_mana_2__agvpt_12305_2615_0__agvpt_12245_-4889_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agvpt_12305_2615_0))
  :effect (and (explored ptobs_7739_755) (explored ptobs_12420_-5014) (explored ptobs_12070_2605) (explored ptobs_10399_-3025) (at-r mana agvpt_12245_-4889_0))
  
  :methods(

      :method patrol_mana_2__agvpt_12305_2615_0__agvpt_12245_-4889_0_d_2
      :actions (move-0 (move mana agvpt_12305_2615_0 agvpt_9720_1934_0))
        (move-1 (move mana agvpt_9720_1934_0 agvpt_8024_909_0))
        (move-2 (move mana agvpt_8024_909_0 agvpt_8964_-869_0))
        (move-3 (move mana agvpt_8964_-869_0 agvpt_10305_-2744_0))
        (move-4 (move mana agvpt_10305_-2744_0 agvpt_12245_-4889_0))
        (explore-0 (observe mana agvpt_12305_2615_0 ptobs_12070_2605))
        (explore-1 (observe mana agvpt_8024_909_0 ptobs_7739_755))
        (explore-2 (observe mana agvpt_10305_-2744_0 ptobs_10399_-3025))
        (explore-3 (observe mana agvpt_12245_-4889_0 ptobs_12420_-5014))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agvpt_12305_2615_0))
        (move-0 move-1 (at-r mana agvpt_9720_1934_0))
        (move-1 move-2 (at-r mana agvpt_8024_909_0))
        (move-2 move-3 (at-r mana agvpt_8964_-869_0))
        (move-3 move-4 (at-r mana agvpt_10305_-2744_0))
        (move-4 :goal (at-r mana agvpt_12245_-4889_0))
        (explore-0 :goal (explored ptobs_12070_2605))
        (:init explore-0 (at-r mana agvpt_12305_2615_0))
        (explore-1 :goal (explored ptobs_7739_755))
        (move-1 explore-1 (at-r mana agvpt_8024_909_0))
        (explore-2 :goal (explored ptobs_10399_-3025))
        (move-3 explore-2 (at-r mana agvpt_10305_-2744_0))
        (explore-3 :goal (explored ptobs_12420_-5014))
        (move-4 explore-3 (at-r mana agvpt_12245_-4889_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-2)
        (explore-2 move-4)

  )
)


(:action patrol_mana_2__agvpt_12245_-4889_0__agvpt_12305_2615_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agvpt_12245_-4889_0))
  :effect (and (explored ptobs_7739_755) (explored ptobs_12420_-5014) (explored ptobs_12070_2605) (explored ptobs_10399_-3025) (at-r mana agvpt_12305_2615_0))
  
  :methods(

      :method patrol_mana_2__agvpt_12245_-4889_0__agvpt_12305_2615_0_i_2
      :actions (move-0 (move mana agvpt_12245_-4889_0 agvpt_10305_-2744_0))
        (move-1 (move mana agvpt_10305_-2744_0 agvpt_8964_-869_0))
        (move-2 (move mana agvpt_8964_-869_0 agvpt_8024_909_0))
        (move-3 (move mana agvpt_8024_909_0 agvpt_9720_1934_0))
        (move-4 (move mana agvpt_9720_1934_0 agvpt_12305_2615_0))
        (explore-0 (observe mana agvpt_12245_-4889_0 ptobs_12420_-5014))
        (explore-1 (observe mana agvpt_10305_-2744_0 ptobs_10399_-3025))
        (explore-2 (observe mana agvpt_8024_909_0 ptobs_7739_755))
        (explore-3 (observe mana agvpt_12305_2615_0 ptobs_12070_2605))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agvpt_12245_-4889_0))
        (move-0 move-1 (at-r mana agvpt_10305_-2744_0))
        (move-1 move-2 (at-r mana agvpt_8964_-869_0))
        (move-2 move-3 (at-r mana agvpt_8024_909_0))
        (move-3 move-4 (at-r mana agvpt_9720_1934_0))
        (move-4 :goal (at-r mana agvpt_12305_2615_0))
        (explore-0 :goal (explored ptobs_12420_-5014))
        (:init explore-0 (at-r mana agvpt_12245_-4889_0))
        (explore-1 :goal (explored ptobs_10399_-3025))
        (move-0 explore-1 (at-r mana agvpt_10305_-2744_0))
        (explore-2 :goal (explored ptobs_7739_755))
        (move-2 explore-2 (at-r mana agvpt_8024_909_0))
        (explore-3 :goal (explored ptobs_12070_2605))
        (move-4 explore-3 (at-r mana agvpt_12305_2615_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)
        (explore-2 move-3)

  )
)


(:action patrol_mana_3__agvpt_21300_2230_0__agvpt_19270_-6544_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agvpt_21300_2230_0))
  :effect (and (explored ptobs_23834_-1100) (explored ptobs_19229_-6710) (at-r mana agvpt_19270_-6544_0))
  
  :methods(

      :method patrol_mana_3__agvpt_21300_2230_0__agvpt_19270_-6544_0_d_3
      :actions (move-0 (move mana agvpt_21300_2230_0 agvpt_22406_1138_0))
        (move-1 (move mana agvpt_22406_1138_0 agvpt_23759_-905_0))
        (move-2 (move mana agvpt_23759_-905_0 agvpt_21284_-4139_0))
        (move-3 (move mana agvpt_21284_-4139_0 agvpt_19270_-6544_0))
        (explore-0 (observe mana agvpt_23759_-905_0 ptobs_23834_-1100))
        (explore-1 (observe mana agvpt_19270_-6544_0 ptobs_19229_-6710))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agvpt_21300_2230_0))
        (move-0 move-1 (at-r mana agvpt_22406_1138_0))
        (move-1 move-2 (at-r mana agvpt_23759_-905_0))
        (move-2 move-3 (at-r mana agvpt_21284_-4139_0))
        (move-3 :goal (at-r mana agvpt_19270_-6544_0))
        (explore-0 :goal (explored ptobs_23834_-1100))
        (move-1 explore-0 (at-r mana agvpt_23759_-905_0))
        (explore-1 :goal (explored ptobs_19229_-6710))
        (move-3 explore-1 (at-r mana agvpt_19270_-6544_0))
      :temporal-links (explore-0 move-2)

  )
)


(:action patrol_mana_3__agvpt_19270_-6544_0__agvpt_21300_2230_0
  :parameters ()
  :conflict-with (at-r mana *)
  
  :precondition (and (at-r mana agvpt_19270_-6544_0))
  :effect (and (explored ptobs_23834_-1100) (explored ptobs_19229_-6710) (at-r mana agvpt_21300_2230_0))
  
  :methods(

      :method patrol_mana_3__agvpt_19270_-6544_0__agvpt_21300_2230_0_i_3
      :actions (move-0 (move mana agvpt_19270_-6544_0 agvpt_21284_-4139_0))
        (move-1 (move mana agvpt_21284_-4139_0 agvpt_23759_-905_0))
        (move-2 (move mana agvpt_23759_-905_0 agvpt_22406_1138_0))
        (move-3 (move mana agvpt_22406_1138_0 agvpt_21300_2230_0))
        (explore-0 (observe mana agvpt_19270_-6544_0 ptobs_19229_-6710))
        (explore-1 (observe mana agvpt_23759_-905_0 ptobs_23834_-1100))
      
      :precondition
      :causal-links (:init move-0 (at-r mana agvpt_19270_-6544_0))
        (move-0 move-1 (at-r mana agvpt_21284_-4139_0))
        (move-1 move-2 (at-r mana agvpt_23759_-905_0))
        (move-2 move-3 (at-r mana agvpt_22406_1138_0))
        (move-3 :goal (at-r mana agvpt_21300_2230_0))
        (explore-0 :goal (explored ptobs_19229_-6710))
        (:init explore-0 (at-r mana agvpt_19270_-6544_0))
        (explore-1 :goal (explored ptobs_23834_-1100))
        (move-1 explore-1 (at-r mana agvpt_23759_-905_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-2)

  )
)


(:action patrol_minnie_1__agvpt_20942_2602_0__agvpt_12305_2615_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agvpt_20942_2602_0))
  :effect (and (explored ptobs_19464_4275) (explored ptobs_12070_2605) (explored ptobs_14889_4915) (at-r minnie agvpt_12305_2615_0))
  
  :methods(

      :method patrol_minnie_1__agvpt_20942_2602_0__agvpt_12305_2615_0_d_1
      :actions (move-0 (move minnie agvpt_20942_2602_0 agvpt_19478_4554_0))
        (move-1 (move minnie agvpt_19478_4554_0 agvpt_18502_5530_0))
        (move-2 (move minnie agvpt_18502_5530_0 agvpt_17037_6506_0))
        (move-3 (move minnie agvpt_17037_6506_0 agvpt_15429_5345_0))
        (move-4 (move minnie agvpt_15429_5345_0 agvpt_14109_4066_0))
        (move-5 (move minnie agvpt_14109_4066_0 agvpt_12305_2615_0))
        (explore-0 (observe minnie agvpt_19478_4554_0 ptobs_19464_4275))
        (explore-1 (observe minnie agvpt_15429_5345_0 ptobs_14889_4915))
        (explore-2 (observe minnie agvpt_12305_2615_0 ptobs_12070_2605))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agvpt_20942_2602_0))
        (move-0 move-1 (at-r minnie agvpt_19478_4554_0))
        (move-1 move-2 (at-r minnie agvpt_18502_5530_0))
        (move-2 move-3 (at-r minnie agvpt_17037_6506_0))
        (move-3 move-4 (at-r minnie agvpt_15429_5345_0))
        (move-4 move-5 (at-r minnie agvpt_14109_4066_0))
        (move-5 :goal (at-r minnie agvpt_12305_2615_0))
        (explore-0 :goal (explored ptobs_19464_4275))
        (move-0 explore-0 (at-r minnie agvpt_19478_4554_0))
        (explore-1 :goal (explored ptobs_14889_4915))
        (move-3 explore-1 (at-r minnie agvpt_15429_5345_0))
        (explore-2 :goal (explored ptobs_12070_2605))
        (move-5 explore-2 (at-r minnie agvpt_12305_2615_0))
      :temporal-links (explore-0 move-1)
        (explore-1 move-4)

  )
)


(:action patrol_minnie_1__agvpt_12305_2615_0__agvpt_20942_2602_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agvpt_12305_2615_0))
  :effect (and (explored ptobs_19464_4275) (explored ptobs_12070_2605) (explored ptobs_14889_4915) (at-r minnie agvpt_20942_2602_0))
  
  :methods(

      :method patrol_minnie_1__agvpt_12305_2615_0__agvpt_20942_2602_0_i_1
      :actions (move-0 (move minnie agvpt_12305_2615_0 agvpt_14109_4066_0))
        (move-1 (move minnie agvpt_14109_4066_0 agvpt_15429_5345_0))
        (move-2 (move minnie agvpt_15429_5345_0 agvpt_17037_6506_0))
        (move-3 (move minnie agvpt_17037_6506_0 agvpt_18502_5530_0))
        (move-4 (move minnie agvpt_18502_5530_0 agvpt_19478_4554_0))
        (move-5 (move minnie agvpt_19478_4554_0 agvpt_20942_2602_0))
        (explore-0 (observe minnie agvpt_12305_2615_0 ptobs_12070_2605))
        (explore-1 (observe minnie agvpt_15429_5345_0 ptobs_14889_4915))
        (explore-2 (observe minnie agvpt_19478_4554_0 ptobs_19464_4275))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agvpt_12305_2615_0))
        (move-0 move-1 (at-r minnie agvpt_14109_4066_0))
        (move-1 move-2 (at-r minnie agvpt_15429_5345_0))
        (move-2 move-3 (at-r minnie agvpt_17037_6506_0))
        (move-3 move-4 (at-r minnie agvpt_18502_5530_0))
        (move-4 move-5 (at-r minnie agvpt_19478_4554_0))
        (move-5 :goal (at-r minnie agvpt_20942_2602_0))
        (explore-0 :goal (explored ptobs_12070_2605))
        (:init explore-0 (at-r minnie agvpt_12305_2615_0))
        (explore-1 :goal (explored ptobs_14889_4915))
        (move-1 explore-1 (at-r minnie agvpt_15429_5345_0))
        (explore-2 :goal (explored ptobs_19464_4275))
        (move-4 explore-2 (at-r minnie agvpt_19478_4554_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-2)
        (explore-2 move-5)

  )
)


(:action patrol_minnie_4__agvpt_19270_-6544_0__agvpt_12245_-4889_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agvpt_19270_-6544_0))
  :effect (and (explored ptobs_12420_-5014) (explored ptobs_15645_-10244) (explored ptobs_19229_-6710) (at-r minnie agvpt_12245_-4889_0))
  
  :methods(

      :method patrol_minnie_4__agvpt_19270_-6544_0__agvpt_12245_-4889_0_d_4
      :actions (move-0 (move minnie agvpt_19270_-6544_0 agvpt_17525_-8623_0))
        (move-1 (move minnie agvpt_17525_-8623_0 agvpt_16061_-10087_0))
        (move-2 (move minnie agvpt_16061_-10087_0 agvpt_13539_-6550_0))
        (move-3 (move minnie agvpt_13539_-6550_0 agvpt_12245_-4889_0))
        (explore-0 (observe minnie agvpt_19270_-6544_0 ptobs_19229_-6710))
        (explore-1 (observe minnie agvpt_16061_-10087_0 ptobs_15645_-10244))
        (explore-2 (observe minnie agvpt_12245_-4889_0 ptobs_12420_-5014))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agvpt_19270_-6544_0))
        (move-0 move-1 (at-r minnie agvpt_17525_-8623_0))
        (move-1 move-2 (at-r minnie agvpt_16061_-10087_0))
        (move-2 move-3 (at-r minnie agvpt_13539_-6550_0))
        (move-3 :goal (at-r minnie agvpt_12245_-4889_0))
        (explore-0 :goal (explored ptobs_19229_-6710))
        (:init explore-0 (at-r minnie agvpt_19270_-6544_0))
        (explore-1 :goal (explored ptobs_15645_-10244))
        (move-1 explore-1 (at-r minnie agvpt_16061_-10087_0))
        (explore-2 :goal (explored ptobs_12420_-5014))
        (move-3 explore-2 (at-r minnie agvpt_12245_-4889_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-2)

  )
)


(:action patrol_minnie_4__agvpt_12245_-4889_0__agvpt_19270_-6544_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agvpt_12245_-4889_0))
  :effect (and (explored ptobs_12420_-5014) (explored ptobs_15645_-10244) (explored ptobs_19229_-6710) (at-r minnie agvpt_19270_-6544_0))
  
  :methods(

      :method patrol_minnie_4__agvpt_12245_-4889_0__agvpt_19270_-6544_0_i_4
      :actions (move-0 (move minnie agvpt_12245_-4889_0 agvpt_13539_-6550_0))
        (move-1 (move minnie agvpt_13539_-6550_0 agvpt_16061_-10087_0))
        (move-2 (move minnie agvpt_16061_-10087_0 agvpt_17525_-8623_0))
        (move-3 (move minnie agvpt_17525_-8623_0 agvpt_19270_-6544_0))
        (explore-0 (observe minnie agvpt_12245_-4889_0 ptobs_12420_-5014))
        (explore-1 (observe minnie agvpt_16061_-10087_0 ptobs_15645_-10244))
        (explore-2 (observe minnie agvpt_19270_-6544_0 ptobs_19229_-6710))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agvpt_12245_-4889_0))
        (move-0 move-1 (at-r minnie agvpt_13539_-6550_0))
        (move-1 move-2 (at-r minnie agvpt_16061_-10087_0))
        (move-2 move-3 (at-r minnie agvpt_17525_-8623_0))
        (move-3 :goal (at-r minnie agvpt_19270_-6544_0))
        (explore-0 :goal (explored ptobs_12420_-5014))
        (:init explore-0 (at-r minnie agvpt_12245_-4889_0))
        (explore-1 :goal (explored ptobs_15645_-10244))
        (move-1 explore-1 (at-r minnie agvpt_16061_-10087_0))
        (explore-2 :goal (explored ptobs_19229_-6710))
        (move-3 explore-2 (at-r minnie agvpt_19270_-6544_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-2)

  )
)


(:action patrol_minnie_2__agvpt_12305_2615_0__agvpt_12245_-4889_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agvpt_12305_2615_0))
  :effect (and (explored ptobs_7739_755) (explored ptobs_12420_-5014) (explored ptobs_12070_2605) (explored ptobs_10399_-3025) (at-r minnie agvpt_12245_-4889_0))
  
  :methods(

      :method patrol_minnie_2__agvpt_12305_2615_0__agvpt_12245_-4889_0_d_2
      :actions (move-0 (move minnie agvpt_12305_2615_0 agvpt_9720_1934_0))
        (move-1 (move minnie agvpt_9720_1934_0 agvpt_8024_909_0))
        (move-2 (move minnie agvpt_8024_909_0 agvpt_8964_-869_0))
        (move-3 (move minnie agvpt_8964_-869_0 agvpt_10305_-2744_0))
        (move-4 (move minnie agvpt_10305_-2744_0 agvpt_12245_-4889_0))
        (explore-0 (observe minnie agvpt_12305_2615_0 ptobs_12070_2605))
        (explore-1 (observe minnie agvpt_8024_909_0 ptobs_7739_755))
        (explore-2 (observe minnie agvpt_10305_-2744_0 ptobs_10399_-3025))
        (explore-3 (observe minnie agvpt_12245_-4889_0 ptobs_12420_-5014))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agvpt_12305_2615_0))
        (move-0 move-1 (at-r minnie agvpt_9720_1934_0))
        (move-1 move-2 (at-r minnie agvpt_8024_909_0))
        (move-2 move-3 (at-r minnie agvpt_8964_-869_0))
        (move-3 move-4 (at-r minnie agvpt_10305_-2744_0))
        (move-4 :goal (at-r minnie agvpt_12245_-4889_0))
        (explore-0 :goal (explored ptobs_12070_2605))
        (:init explore-0 (at-r minnie agvpt_12305_2615_0))
        (explore-1 :goal (explored ptobs_7739_755))
        (move-1 explore-1 (at-r minnie agvpt_8024_909_0))
        (explore-2 :goal (explored ptobs_10399_-3025))
        (move-3 explore-2 (at-r minnie agvpt_10305_-2744_0))
        (explore-3 :goal (explored ptobs_12420_-5014))
        (move-4 explore-3 (at-r minnie agvpt_12245_-4889_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-2)
        (explore-2 move-4)

  )
)


(:action patrol_minnie_2__agvpt_12245_-4889_0__agvpt_12305_2615_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agvpt_12245_-4889_0))
  :effect (and (explored ptobs_7739_755) (explored ptobs_12420_-5014) (explored ptobs_12070_2605) (explored ptobs_10399_-3025) (at-r minnie agvpt_12305_2615_0))
  
  :methods(

      :method patrol_minnie_2__agvpt_12245_-4889_0__agvpt_12305_2615_0_i_2
      :actions (move-0 (move minnie agvpt_12245_-4889_0 agvpt_10305_-2744_0))
        (move-1 (move minnie agvpt_10305_-2744_0 agvpt_8964_-869_0))
        (move-2 (move minnie agvpt_8964_-869_0 agvpt_8024_909_0))
        (move-3 (move minnie agvpt_8024_909_0 agvpt_9720_1934_0))
        (move-4 (move minnie agvpt_9720_1934_0 agvpt_12305_2615_0))
        (explore-0 (observe minnie agvpt_12245_-4889_0 ptobs_12420_-5014))
        (explore-1 (observe minnie agvpt_10305_-2744_0 ptobs_10399_-3025))
        (explore-2 (observe minnie agvpt_8024_909_0 ptobs_7739_755))
        (explore-3 (observe minnie agvpt_12305_2615_0 ptobs_12070_2605))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agvpt_12245_-4889_0))
        (move-0 move-1 (at-r minnie agvpt_10305_-2744_0))
        (move-1 move-2 (at-r minnie agvpt_8964_-869_0))
        (move-2 move-3 (at-r minnie agvpt_8024_909_0))
        (move-3 move-4 (at-r minnie agvpt_9720_1934_0))
        (move-4 :goal (at-r minnie agvpt_12305_2615_0))
        (explore-0 :goal (explored ptobs_12420_-5014))
        (:init explore-0 (at-r minnie agvpt_12245_-4889_0))
        (explore-1 :goal (explored ptobs_10399_-3025))
        (move-0 explore-1 (at-r minnie agvpt_10305_-2744_0))
        (explore-2 :goal (explored ptobs_7739_755))
        (move-2 explore-2 (at-r minnie agvpt_8024_909_0))
        (explore-3 :goal (explored ptobs_12070_2605))
        (move-4 explore-3 (at-r minnie agvpt_12305_2615_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-1)
        (explore-2 move-3)

  )
)


(:action patrol_minnie_3__agvpt_21300_2230_0__agvpt_19270_-6544_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agvpt_21300_2230_0))
  :effect (and (explored ptobs_23834_-1100) (explored ptobs_19229_-6710) (at-r minnie agvpt_19270_-6544_0))
  
  :methods(

      :method patrol_minnie_3__agvpt_21300_2230_0__agvpt_19270_-6544_0_d_3
      :actions (move-0 (move minnie agvpt_21300_2230_0 agvpt_22406_1138_0))
        (move-1 (move minnie agvpt_22406_1138_0 agvpt_23759_-905_0))
        (move-2 (move minnie agvpt_23759_-905_0 agvpt_21284_-4139_0))
        (move-3 (move minnie agvpt_21284_-4139_0 agvpt_19270_-6544_0))
        (explore-0 (observe minnie agvpt_23759_-905_0 ptobs_23834_-1100))
        (explore-1 (observe minnie agvpt_19270_-6544_0 ptobs_19229_-6710))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agvpt_21300_2230_0))
        (move-0 move-1 (at-r minnie agvpt_22406_1138_0))
        (move-1 move-2 (at-r minnie agvpt_23759_-905_0))
        (move-2 move-3 (at-r minnie agvpt_21284_-4139_0))
        (move-3 :goal (at-r minnie agvpt_19270_-6544_0))
        (explore-0 :goal (explored ptobs_23834_-1100))
        (move-1 explore-0 (at-r minnie agvpt_23759_-905_0))
        (explore-1 :goal (explored ptobs_19229_-6710))
        (move-3 explore-1 (at-r minnie agvpt_19270_-6544_0))
      :temporal-links (explore-0 move-2)

  )
)


(:action patrol_minnie_3__agvpt_19270_-6544_0__agvpt_21300_2230_0
  :parameters ()
  :conflict-with (at-r minnie *)
  
  :precondition (and (at-r minnie agvpt_19270_-6544_0))
  :effect (and (explored ptobs_23834_-1100) (explored ptobs_19229_-6710) (at-r minnie agvpt_21300_2230_0))
  
  :methods(

      :method patrol_minnie_3__agvpt_19270_-6544_0__agvpt_21300_2230_0_i_3
      :actions (move-0 (move minnie agvpt_19270_-6544_0 agvpt_21284_-4139_0))
        (move-1 (move minnie agvpt_21284_-4139_0 agvpt_23759_-905_0))
        (move-2 (move minnie agvpt_23759_-905_0 agvpt_22406_1138_0))
        (move-3 (move minnie agvpt_22406_1138_0 agvpt_21300_2230_0))
        (explore-0 (observe minnie agvpt_19270_-6544_0 ptobs_19229_-6710))
        (explore-1 (observe minnie agvpt_23759_-905_0 ptobs_23834_-1100))
      
      :precondition
      :causal-links (:init move-0 (at-r minnie agvpt_19270_-6544_0))
        (move-0 move-1 (at-r minnie agvpt_21284_-4139_0))
        (move-1 move-2 (at-r minnie agvpt_23759_-905_0))
        (move-2 move-3 (at-r minnie agvpt_22406_1138_0))
        (move-3 :goal (at-r minnie agvpt_21300_2230_0))
        (explore-0 :goal (explored ptobs_19229_-6710))
        (:init explore-0 (at-r minnie agvpt_19270_-6544_0))
        (explore-1 :goal (explored ptobs_23834_-1100))
        (move-1 explore-1 (at-r minnie agvpt_23759_-905_0))
      :temporal-links (explore-0 move-0)
        (explore-1 move-2)

  )
)


(:action patrol_ressac1_1__ressac1pt_16239_5397_0__ressac1pt_12245_1404_0
  :parameters ()
  :conflict-with (at-r ressac1 *)
  
  :precondition (and (at-r ressac1 ressac1pt_16239_5397_0))
  :effect (and (explored ptobs_13799_2820) (at-r ressac1 ressac1pt_12245_1404_0))
  
  :methods(

      :method patrol_ressac1_1__ressac1pt_16239_5397_0__ressac1pt_12245_1404_0_d_1
      :actions (move-0 (move ressac1 ressac1pt_16239_5397_0 ressac1pt_18235_5397_0))
        (move-1 (move ressac1 ressac1pt_18235_5397_0 ressac1pt_18235_1404_0))
        (move-2 (move ressac1 ressac1pt_18235_1404_0 ressac1pt_16184_-5_0))
        (move-3 (move ressac1 ressac1pt_16184_-5_0 ressac1pt_13644_3045_0))
        (move-4 (move ressac1 ressac1pt_13644_3045_0 ressac1pt_12245_1404_0))
        (explore-0 (observe ressac1 ressac1pt_13644_3045_0 ptobs_13799_2820))
      
      :precondition
      :causal-links (:init move-0 (at-r ressac1 ressac1pt_16239_5397_0))
        (move-0 move-1 (at-r ressac1 ressac1pt_18235_5397_0))
        (move-1 move-2 (at-r ressac1 ressac1pt_18235_1404_0))
        (move-2 move-3 (at-r ressac1 ressac1pt_16184_-5_0))
        (move-3 move-4 (at-r ressac1 ressac1pt_13644_3045_0))
        (move-4 :goal (at-r ressac1 ressac1pt_12245_1404_0))
        (explore-0 :goal (explored ptobs_13799_2820))
        (move-3 explore-0 (at-r ressac1 ressac1pt_13644_3045_0))
      :temporal-links (explore-0 move-4)

  )
)


(:action patrol_ressac1_1__ressac1pt_12245_1404_0__ressac1pt_16239_5397_0
  :parameters ()
  :conflict-with (at-r ressac1 *)
  
  :precondition (and (at-r ressac1 ressac1pt_12245_1404_0))
  :effect (and (explored ptobs_13799_2820) (at-r ressac1 ressac1pt_16239_5397_0))
  
  :methods(

      :method patrol_ressac1_1__ressac1pt_12245_1404_0__ressac1pt_16239_5397_0_i_1
      :actions (move-0 (move ressac1 ressac1pt_12245_1404_0 ressac1pt_13644_3045_0))
        (move-1 (move ressac1 ressac1pt_13644_3045_0 ressac1pt_16184_-5_0))
        (move-2 (move ressac1 ressac1pt_16184_-5_0 ressac1pt_18235_1404_0))
        (move-3 (move ressac1 ressac1pt_18235_1404_0 ressac1pt_18235_5397_0))
        (move-4 (move ressac1 ressac1pt_18235_5397_0 ressac1pt_16239_5397_0))
        (explore-0 (observe ressac1 ressac1pt_13644_3045_0 ptobs_13799_2820))
      
      :precondition
      :causal-links (:init move-0 (at-r ressac1 ressac1pt_12245_1404_0))
        (move-0 move-1 (at-r ressac1 ressac1pt_13644_3045_0))
        (move-1 move-2 (at-r ressac1 ressac1pt_16184_-5_0))
        (move-2 move-3 (at-r ressac1 ressac1pt_18235_1404_0))
        (move-3 move-4 (at-r ressac1 ressac1pt_18235_5397_0))
        (move-4 :goal (at-r ressac1 ressac1pt_16239_5397_0))
        (explore-0 :goal (explored ptobs_13799_2820))
        (move-0 explore-0 (at-r ressac1 ressac1pt_13644_3045_0))
      :temporal-links (explore-0 move-1)

  )
)


(:action patrol_ressac1_2__ressac1pt_12245_1404_0__ressac1pt_7945_619_0
  :parameters ()
  :conflict-with (at-r ressac1 *)
  
  :precondition (and (at-r ressac1 ressac1pt_12245_1404_0))
  :effect (and (explored ptobs_14104_-725) (explored ptobs_10399_-3025) (at-r ressac1 ressac1pt_7945_619_0))
  
  :methods(

      :method patrol_ressac1_2__ressac1pt_12245_1404_0__ressac1pt_7945_619_0_d_2
      :actions (move-0 (move ressac1 ressac1pt_12245_1404_0 ressac1pt_14170_-509_0))
        (move-1 (move ressac1 ressac1pt_14170_-509_0 ressac1pt_10249_-2588_0))
        (move-2 (move ressac1 ressac1pt_10249_-2588_0 ressac1pt_7945_619_0))
        (explore-0 (observe ressac1 ressac1pt_14170_-509_0 ptobs_14104_-725))
        (explore-1 (observe ressac1 ressac1pt_10249_-2588_0 ptobs_10399_-3025))
      
      :precondition
      :causal-links (:init move-0 (at-r ressac1 ressac1pt_12245_1404_0))
        (move-0 move-1 (at-r ressac1 ressac1pt_14170_-509_0))
        (move-1 move-2 (at-r ressac1 ressac1pt_10249_-2588_0))
        (move-2 :goal (at-r ressac1 ressac1pt_7945_619_0))
        (explore-0 :goal (explored ptobs_14104_-725))
        (move-0 explore-0 (at-r ressac1 ressac1pt_14170_-509_0))
        (explore-1 :goal (explored ptobs_10399_-3025))
        (move-1 explore-1 (at-r ressac1 ressac1pt_10249_-2588_0))
      :temporal-links (explore-0 move-1)
        (explore-1 move-2)

  )
)


(:action patrol_ressac1_2__ressac1pt_7945_619_0__ressac1pt_12245_1404_0
  :parameters ()
  :conflict-with (at-r ressac1 *)
  
  :precondition (and (at-r ressac1 ressac1pt_7945_619_0))
  :effect (and (explored ptobs_14104_-725) (explored ptobs_10399_-3025) (at-r ressac1 ressac1pt_12245_1404_0))
  
  :methods(

      :method patrol_ressac1_2__ressac1pt_7945_619_0__ressac1pt_12245_1404_0_i_2
      :actions (move-0 (move ressac1 ressac1pt_7945_619_0 ressac1pt_10249_-2588_0))
        (move-1 (move ressac1 ressac1pt_10249_-2588_0 ressac1pt_14170_-509_0))
        (move-2 (move ressac1 ressac1pt_14170_-509_0 ressac1pt_12245_1404_0))
        (explore-0 (observe ressac1 ressac1pt_10249_-2588_0 ptobs_10399_-3025))
        (explore-1 (observe ressac1 ressac1pt_14170_-509_0 ptobs_14104_-725))
      
      :precondition
      :causal-links (:init move-0 (at-r ressac1 ressac1pt_7945_619_0))
        (move-0 move-1 (at-r ressac1 ressac1pt_10249_-2588_0))
        (move-1 move-2 (at-r ressac1 ressac1pt_14170_-509_0))
        (move-2 :goal (at-r ressac1 ressac1pt_12245_1404_0))
        (explore-0 :goal (explored ptobs_10399_-3025))
        (move-0 explore-0 (at-r ressac1 ressac1pt_10249_-2588_0))
        (explore-1 :goal (explored ptobs_14104_-725))
        (move-1 explore-1 (at-r ressac1 ressac1pt_14170_-509_0))
      :temporal-links (explore-0 move-1)
        (explore-1 move-2)

  )
)

)
