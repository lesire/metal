; Plan generated
; 0.000: (dummy init) [0.000]
0.001: (observe r2 wp_a2 obs_a2) [0.500]
0.001: (observe r3 wp_a3 obs_a3) [0.500]
0.001: (observe r4 wp_a4 obs_a4) [0.500]
0.001: (observe r1 wp_a1 obs_a1) [0.500]
0.502: (move r4 wp_a4 wp_b4) [5.000]
0.502: (move r3 wp_a3 wp_b3) [5.000]
0.502: (move r1 wp_a1 wp_b1) [5.000]
0.502: (move r2 wp_a2 wp_b2) [5.000]
5.503: (observe r3 wp_b3 obs_b3) [0.500]
5.503: (observe r4 wp_b4 obs_b4) [0.500]
5.503: (observe r1 wp_b1 obs_b1) [0.500]
5.503: (observe r2 wp_b2 obs_b2) [0.500]
6.004: (move r1 wp_b1 wp_c1) [5.000]
6.004: (move r3 wp_b3 wp_c3) [5.000]
6.004: (move r2 wp_b2 wp_com) [5.000]
6.004: (move r4 wp_b4 wp_com) [5.000]
11.005: (observe r3 wp_c3 obs_c3) [0.500]
11.005: (observe r1 wp_c1 obs_c1) [0.500]
11.506: (move r1 wp_c1 wp_a1) [5.000]
11.506: (move r3 wp_c3 wp_a3) [5.000]
16.507: (move r3 wp_a3 wp_com) [5.000]
16.507: (move r1 wp_a1 wp_com) [5.000]
24.501: (communicate r1 r2 wp_com wp_com) [1.000]
24.501: (communicate r2 r1 wp_com wp_com) [1.000]
24.501: (communicate r3 r4 wp_com wp_com) [1.000]
24.501: (communicate r4 r3 wp_com wp_com) [1.000]
25.000: (communicate-meta r1 r2 wp_com wp_com) [0.500]
25.000: (communicate-meta r3 r4 wp_com wp_com) [0.500]
25.502: (move r1 wp_com wp_d1) [5.000]
25.502: (move r4 wp_com wp_d4) [5.000]
25.502: (move r3 wp_com wp_d3) [5.000]
25.502: (move r2 wp_com wp_d2) [5.000]
30.503: (observe r4 wp_d4 obs_d4) [0.500]
30.503: (observe r3 wp_d3 obs_d3) [0.500]
30.503: (observe r1 wp_d1 obs_d1) [0.500]
30.503: (observe r2 wp_d2 obs_d2) [0.500]
; 31.004: (dummy end) [0.000]
