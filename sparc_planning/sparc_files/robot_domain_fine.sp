%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% AutoGenerated SPARC file
%% Author: MARK ROBSON 2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#const numSteps = 1.

sorts
#robot = {rob0}.
#thing = {}.
#object = #robot + #thing.
#grasp_mode = {dexterous,vacuum}.
#place_c = {input_area,intermediate_area,assembly_area}.
#table_locs = {c1,c2,c3,c4,c5,c6}.
#non_placement_location = {above_input,above_intermediate,above_assembly}.
#place_f = #table_locs + #non_placement_location.
#fine_res_sort = #place_f.
#coarse_res_sort = #place_c.
#action = putdown(#robot,#thing), move_f(#robot,#place_f), pick_up(#robot,#thing), change_grasp_mode(#robot,#grasp_mode).
#boolean = {true, false}.
#outcome = {true, false, undet}.
#defined_fluent = {}.
#inertial_fluent = in_hand(#robot, #thing)+ loc_c(#object, #place_c)+ loc_f(#object, #place_f)+ current_grasp_mode(#robot, #grasp_mode)+ on(#thing, #thing)+ clear(#thing).
#fluent = #inertial_fluent + #defined_fluent.
#step = 0..numSteps.

predicates
next_to_c(#place_c, #place_c).
next_to_f(#place_f, #place_f).
component(#coarse_res_sort, #fine_res_sort).
holds(#fluent, #boolean, #step).
occurs(#action, #step).
success().
goal(#step).
something_happened(#step).

rules
-component(C1,P1):- #place_c(P1), not #place_f(C1).
next_to_f(C1,C2):- next_to_f(C2,C1).
-next_to_c(P1,P2):- not next_to_c(P1,P2).
-next_to_c(P1,P2):- not next_to_c(P1,P2).
next_to_c(P1,P2):- next_to_f(C1,C2), component(P1,C1), component(P2,C2).
holds(loc_c(T,P), true, I) :- holds(loc_f(T,C), true, I), component(P,C).
holds(loc_f(T,P1), false, I) :- holds(loc_f(T,P2), true, I), P1!=P2.
holds(current_grasp_mode(R,G1), false, I) :- holds(current_grasp_mode(R,G2), true, I), G1!=G2.
holds(loc_f(T1,P1), true, I) :- holds(loc_f(T2,P1), true, I), holds(on(T1,T2), true, I).
holds(in_hand(R,T), false, I+1) :- occurs(putdown(R,T), I).
holds(on(T1,T2), true, I+1) :- occurs(putdown(R,T1), I), holds(loc_f(R,P1), true, I), holds(loc_f(T1,P2), true, I), P1=P2, holds(clear(T2), true, I).
-occurs(putdown(R,T), I) :- not holds(in_hand(R,T), true, I).
holds(loc_f(R,P), true, I+1) :- occurs(move_f(R,P), I).
holds(loc_f(T,P), true, I+1) :- occurs(move_f(R,P), I), holds(in_hand(R,T), true, I).
-occurs(move_f(R,P1), I) :- holds(loc_f(R,P2), true, I), P1=P2.
-occurs(move_f(R,P1), I) :- holds(loc_f(R,P2), true, I), not next_to_f(P1,P2).
holds(in_hand(R,T), true, I+1) :- occurs(pick_up(R,T), I).
holds(on(T1,T2), false, I+1) :- occurs(pick_up(R,T1), I).
holds(clear(T2), true, I+1) :- occurs(pick_up(R,T1), I), holds(on(T1,T2), true, I).
-occurs(pick_up(R,T1), I) :- holds(clear(T1), true, I).
holds(current_grasp_mode(R,G), true, I+1) :- occurs(change_grasp_mode(R,G), I).
-occurs(change_grasp_mode(R,G1), I) :- holds(current_grasp_mode(R,G1), true, I), G1=G2.
-occurs(change_grasp_mode(R,G), I) :- holds(in_hand(R,T), true, I).
-holds(F, V2, I) :- holds(F, V1, I), V1!=V2.
holds(F, Y, I+1) :- #inertial_fluent(F), holds(F, Y, I), not -holds(F, Y, I+1), I < numSteps.
-occurs(A,I) :- not occurs(A,I).
success :- goal(I), I <= numSteps.
:- not success.
occurs(A, I) | -occurs(A, I) :- not goal(I).
-occurs(A2, I) :- occurs(A1, I), A1 != A2.
something_happened(I) :- occurs(A, I).
:- not goal(I), not something_happened(I).
goal(I) :- holds(in_hand(rob0,textbook), false, I).
holds(in_hand(rob0,textbook), true, 0).

display
occurs.
