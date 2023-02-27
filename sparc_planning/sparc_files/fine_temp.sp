%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% AutoGenerated SPARC file
%% Author: MARK ROBSON 2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#const numSteps = 7.
#const startStep = 5.

sorts
#in_f_end = {j2,j5,j1,j6}.
#place_c = {assembly_area}.
#beam = {b1,b4,b2,b3}.
#robot = {rob0}.
#angle_m_end = {dummy_angle_m_end}.
#approach_location = {b4a,p1a,b2a,p2a,p3a,p4a,b3a}.
#link = {l4,l3,l1,l2}.
#non_placement_location = {above_assembly_area}.
#in_m_end = {j8,j7,j3,j4}.
#target_location = {b4t,b2t,b3t,p2t,p3t,p1t,p4t}.
#assembly_location = #approach_location + #target_location.
#place_f = #non_placement_location + #assembly_location.
#thing = #beam.
#coarse_res_sort = #place_c + #thing.
#joint = #in_f_end + #in_m_end.
#object = #robot + #thing.
#beam_part = #joint + #link.
#thing_part = #beam_part.
#fine_res_sort = #thing_part + #place_f.
#action = putdown_f(#robot,#thing_part) + move_f(#robot,#place_f) + pick_up_f(#robot,#thing_part) + assemble_f_cap(#robot,#beam_part) + assemble_f_square(#robot,#beam_part).
#boolean = {true, false}.
#outcome = {true, false, undet}.
#inertial_fluent = in_hand_c(#robot, #thing)+ in_hand_f(#robot, #thing_part)+ loc_c(#object, #place_c)+ loc_f(#object, #place_f)+ in_assembly_c(#beam)+ in_assembly_f(#beam_part)+ supported_c(#beam)+ supported_f(#beam_part).
#step = startStep..numSteps.
#fluent = #inertial_fluent.

predicates
next_to_c(#place_c, #place_c).
next_to_f(#place_f, #place_f).
component(#coarse_res_sort, #fine_res_sort).
fits_into_c(#beam, #beam).
fits_into_f(#beam_part, #beam_part).
fits_through_c(#beam, #beam).
fits_through_f(#beam_part, #beam_part).
is_capped_by(#beam, #beam, #beam).
connected_to(#beam_part, #beam_part).
between(#beam_part, #beam_part, #beam_part).
assem_approach_loc(#thing, #approach_location).
assem_target_loc(#thing, #target_location).
holds(#fluent, #boolean, #step).
occurs(#action, #step).
success().
goal(#step).
something_happened(#step).

rules
-component(C1,P1):- #place_c(P1), not #place_f(C1).
-component(C1,P1):- #beam(P1), not #beam_part(C1).
next_to_f(C1,C2):- next_to_f(C2,C1).
-next_to_c(P1,P2):- not next_to_c(P1,P2).
-next_to_f(P1,P2):- not next_to_f(P1,P2).
holds(loc_f(T,P1), false, I) :- holds(loc_f(T,P2), true, I), P1!=P2.
holds(loc_c(T,P1), false, I) :- holds(loc_c(T,P2), true, I), P1!=P2.
next_to_c(P1,P2):- next_to_f(C1,C2), component(P1,C1), component(P2,C2).
holds(loc_c(T,P), true, I) :- holds(loc_f(T,C), true, I), component(P,C).
holds(loc_f(T1,P1), true, I) :- holds(loc_f(R,P1), true, I), holds(in_hand_c(R,T2), true, I), T1=T2.
-component(C1,P1):- #thing(P1), not #thing_part(C1).
holds(in_hand_c(R,T), true, I) :- holds(in_hand_f(R,TP), true, I), component(T,TP).
holds(supported_c(B1), true, I) :- holds(supported_f(P1), true, I), component(B1,P1).
holds(supported_f(P1), true, I) :- holds(supported_c(B1), true, I), component(B1,P1).
holds(in_assembly_c(B1), true, I) :- holds(in_assembly_f(P1), true, I), component(B1,P1).
holds(in_assembly_f(P1), true, I) :- holds(in_assembly_c(B1), true, I), component(B1,P1).
is_capped_by(B1,B2,B3):- fits_into_c(B1,B2), fits_into_c(B1,B3), B2!=B3.
is_capped_by(B1,B2,B3):- is_capped_by(B1,B3,B2).
-is_capped_by(B1,B2,B3):- not is_capped_by(B1,B2,B3).
holds(supported_c(B1), true, I) :- holds(in_assembly_c(B2), true, I), fits_into_c(B1,B2).
holds(supported_c(B1), true, I) :- holds(in_assembly_c(B2), true, I), fits_into_c(B2,B1).
-fits_into_c(B1,B2):- B1=B2.
-fits_through_c(B1,B2):- B1=B2.
-fits_through_c(B1,B2):- fits_into_c(B1,B2).
-fits_into_c(B1,B2):- fits_through_c(B1,B2).
-fits_into_c(B1,B2):- fits_into_c(B2,B1).
-fits_through_c(B1,B2):- fits_through_c(B2,B1).
connected_to(BP1,BP2):- connected_to(BP2,BP1).
-connected_to(BP1,BP2):- not connected_to(BP1,BP2).
between(BP1,BP2,BP3):- connected_to(BP1,BP2), connected_to(BP1,BP3).
between(BP1,BP2,BP3):- connected_to(BP1,BP2), between(BP4,BP1,BP3).
between(BP1,BP2,BP3):- between(BP4,BP1,BP3), between(BP5,BP1,BP3).
between(BP1,BP2,BP3):- between(BP1,BP3,BP2).
fits_into_c(B1,B2):- fits_into_f(D1,D2), component(B1,D1), component(B2,D2).
fits_through_c(B1,B2):- fits_through_f(D1,D2), component(B1,D1), component(B2,D2).

holds(in_hand_f(R,T), false, I+1) :- occurs(putdown_f(R,T), I).
holds(in_hand_c(R,T), false, I+1) :- occurs(putdown_f(R,TP), I), component(T,TP).
-occurs(putdown_f(R,T), I) :- not holds(in_hand_f(R,T), true, I).
-occurs(putdown_f(R,T), I) :- holds(loc_f(R,C), true, I), #non_placement_location(C).

holds(loc_f(R,P), true, I+1) :- occurs(move_f(R,P), I).
-occurs(move_f(R,P1), I) :- holds(loc_f(R,P2), true, I), P1=P2.
-occurs(move_f(R,P1), I) :- holds(loc_f(R,P2), true, I), not next_to_f(P1,P2), P1!=P2.
-occurs(move_f(R,P1), I) :- holds(in_hand_f(R,BP), true, I), holds(in_assembly_f(BP), true, I).
-occurs(move_f(R,P1), I) :- holds(in_hand_c(R,T1), true, I), #target_location(P1).

holds(in_hand_f(R,T), true, I+1) :- occurs(pick_up_f(R,T), I).
-occurs(pick_up_f(R,P), I) :- holds(loc_f(P,P1), true, I), holds(loc_f(R,P2), true, I), P1!=P2.
-occurs(pick_up_f(R,TP), I) :- holds(loc_f(T1,P1), true, I), holds(loc_f(R,P2), true, I), P1!=P2, component(T1,TP).
-occurs(pick_up_f(R,T1), I) :- holds(in_hand_f(rob0,T2), true, I), #thing_part(T2).
-occurs(pick_up_f(R,T1), I) :- component(B,T1), not #link(T1), #beam(B).

holds(loc_f(R,C), true, I+1) :- occurs(assemble_f_cap(R,BP), I), assem_target_loc(B,C), component(B,BP).
holds(in_assembly_f(BP), true, I+1) :- occurs(assemble_f_cap(R,BP), I).
-occurs(assemble_f_cap(R,BP), I) :- is_capped_by(B1,B2,B3), component(B1,BP), not is_capped_by(B4,B1,B5).
-occurs(assemble_f_cap(R,BP), I) :- component(B1,BP), holds(loc_f(R,C1), true, I), assem_approach_loc(B1,C2), C1!=C2.
-occurs(assemble_f_cap(R,BP), I) :- not holds(in_hand_c(R,B), true, I), component(B,BP).
-occurs(assemble_f_cap(R,BP), I) :- holds(in_hand_f(R,BP), true, I).
-occurs(assemble_f_cap(R,BP), I) :- holds(in_assembly_c(B2), true, I), holds(in_assembly_c(B3), true, I), is_capped_by(B1,B2,B3), B2!=B3, component(B1,BP).
-occurs(assemble_f_cap(R,BP), I) :- holds(in_assembly_f(BP2), true, I), fits_through_f(BP2,BP).
-occurs(assemble_f_cap(R,BP), I) :- not holds(in_assembly_c(B2), true, I), holds(in_assembly_c(B3), true, I), is_capped_by(B2,B1,B3), B2!=B3, component(B1,BP).
-occurs(assemble_f_cap(R,BP), I) :- holds(in_assembly_f(BP), true, I).
-occurs(assemble_f_cap(R,BP), I) :- not holds(supported_f(BP), true, I).
-occurs(assemble_f_cap(R,BP), I) :- holds(in_assembly_c(B2), true, I), #beam(B2), holds(loc_f(B2,C1), true, I), assem_target_loc(B2,C2), C1!=C2.
-occurs(assemble_f_cap(R,BP), I) :- holds(loc_f(R,P), true, I), not #assembly_location(P).

holds(loc_f(R,C), true, I+1) :- occurs(assemble_f_square(R,BP), I), assem_target_loc(B,C), component(B,BP), not #angle_m_end(BP).
holds(in_assembly_f(BP), true, I+1) :- occurs(assemble_f_square(R,BP), I), not #angle_m_end(BP).
-occurs(assemble_f_square(R,BP), I) :- is_capped_by(B1,B2,B3), component(B2,BP).
-occurs(assemble_f_square(R,BP), I) :- component(B1,BP), holds(loc_f(R,C1), true, I), assem_approach_loc(B1,C2), C1!=C2.
-occurs(assemble_f_square(R,BP1), I) :- fits_into_f(BP1,BP2), not holds(in_assembly_f(BP2), true, I).
-occurs(assemble_f_square(R,BP), I) :- not holds(in_hand_c(R,B), true, I), component(B,BP).
-occurs(assemble_f_square(R,BP), I) :- holds(in_hand_f(R,BP), true, I).
-occurs(assemble_f_square(R,BP), I) :- holds(in_assembly_c(B2), true, I), holds(in_assembly_c(B3), true, I), is_capped_by(B1,B2,B3), B2!=B3, component(B1,BP).
-occurs(assemble_f_square(R,BP), I) :- holds(in_assembly_f(BP2), true, I), fits_through_f(BP2,BP).
-occurs(assemble_f_square(R,BP), I) :- not holds(in_assembly_c(B2), true, I), holds(in_assembly_c(B3), true, I), is_capped_by(B2,B1,B3), B2!=B3, component(B1,BP).
-occurs(assemble_f_square(R,BP), I) :- holds(in_assembly_f(BP), true, I).
-occurs(assemble_f_square(R,BP), I) :- not holds(supported_f(BP), true, I).
-occurs(assemble_f_square(R,BP), I) :- holds(in_assembly_c(B2), true, I), #beam(B2), holds(loc_f(B2,C1), true, I), assem_target_loc(B2,C2), C1!=C2.
-occurs(assemble_f_square(R,BP), I) :- holds(loc_f(R,P), true, I), not #assembly_location(P).

% planning rules
-holds(F, V2, I) :- holds(F, V1, I), V1!=V2.
holds(F, Y, I+1) :- #inertial_fluent(F), holds(F, Y, I), not -holds(F, Y, I+1), I < numSteps.
-occurs(A,I) :- not occurs(A,I).
success :- goal(I), I <= numSteps.
:- not success.
occurs(A, I) | -occurs(A, I) :- not goal(I).
-occurs(A2, I) :- occurs(A1, I), A1 != A2.
something_happened(I) :- occurs(A, I).
:- not goal(I), not something_happened(I).

% goal definition
goal(I) :- holds(in_assembly_c(b4), true, I).

% domain setup
% coarse next_to location mapping
% coarse beam relations
fits_into_c(b2,b1).
fits_into_c(b3,b1).
fits_into_c(b3,b4).
fits_into_c(b2,b4).
% beam components and connections
connected_to(j1,l1).
connected_to(l1,j2).
component(b1,j1).
component(b1,l1).
component(b1,j2).
connected_to(j3,l2).
connected_to(l2,j4).
component(b2,j3).
component(b2,l2).
component(b2,j4).
connected_to(j5,l3).
connected_to(l3,j6).
component(b3,j5).
component(b3,l3).
component(b3,j6).
connected_to(j7,l4).
connected_to(l4,j8).
component(b4,j7).
component(b4,l4).
component(b4,j8).
% next_to_f location map
next_to_f(above_assembly_area,b2a).
next_to_f(above_assembly_area,b3a).
next_to_f(above_assembly_area,b4a).
next_to_f(above_assembly_area,b2t).
next_to_f(above_assembly_area,b3t).
next_to_f(above_assembly_area,b4t).
next_to_f(above_assembly_area,p1a).
next_to_f(above_assembly_area,p2a).
next_to_f(above_assembly_area,p3a).
next_to_f(above_assembly_area,p4a).
next_to_f(b2t,b2a).
next_to_f(b3t,b3a).
next_to_f(b4t,b4a).
next_to_f(p1t,p1a).
next_to_f(p2t,p2a).
next_to_f(p3t,p3a).
next_to_f(p4t,p4a).
% location component relations
component(assembly_area,above_assembly_area).
component(assembly_area,b2t).
component(assembly_area,b2a).
component(assembly_area,b3t).
component(assembly_area,b3a).
component(assembly_area,b4t).
component(assembly_area,b4a).
component(assembly_area,p1t).
component(assembly_area,p1a).
component(assembly_area,p2t).
component(assembly_area,p2a).
component(assembly_area,p3t).
component(assembly_area,p3a).
component(assembly_area,p4t).
component(assembly_area,p4a).
% fine beam to beam connections
fits_into_f(j3,j1).
fits_into_f(j5,j2).
fits_into_f(j6,j8).
fits_into_f(j4,j7).
% beam assembly location mapping
assem_approach_loc(b2,b2a).
assem_approach_loc(b3,b3a).
assem_approach_loc(b4,b4a).
assem_target_loc(b2,b2t).
assem_target_loc(b3,b3t).
assem_target_loc(b4,b4t).
% pin assembly location mapping
% near to mapping
-holds(in_hand_f(rob0,l4),false,5).
-holds(in_assembly_f(j5),false,5).
-holds(in_assembly_f(j4),false,5).
-holds(in_assembly_f(j6),false,5).
-holds(in_assembly_f(j1),false,5).
-holds(in_assembly_f(j3),false,5).
-holds(in_assembly_f(j2),false,5).
-holds(in_assembly_f(l1),false,5).
-holds(in_assembly_f(l3),false,5).
-holds(in_assembly_f(l2),false,5).
-holds(in_assembly_c(b2),false,5).
-holds(in_assembly_c(b3),false,5).
-holds(in_assembly_c(b1),false,5).
-holds(in_hand_c(rob0,b4),false,5).
-holds(loc_f(b2,b2t),false,5).
-holds(loc_f(b3,b3t),false,5).
-holds(loc_c(b3,assembly_area),false,5).
-holds(loc_c(b1,assembly_area),false,5).
-holds(loc_c(b2,assembly_area),false,5).
-holds(supported_c(b3),false,5).
-holds(supported_c(b2),false,5).
-holds(supported_c(b4),false,5).
-holds(supported_c(b1),false,5).
-holds(supported_f(j2),false,5).
-holds(supported_f(l1),false,5).
-holds(supported_f(j1),false,5).
-holds(supported_f(j8),false,5).
-holds(supported_f(l4),false,5).
-holds(supported_f(j7),false,5).
-holds(supported_f(j4),false,5).
-holds(supported_f(l2),false,5).
-holds(supported_f(j3),false,5).
-holds(supported_f(j6),false,5).
-holds(supported_f(l3),false,5).
-holds(supported_f(j5),false,5).
-holds(in_hand_f(rob0,l1),true,5).
-holds(in_hand_f(rob0,l2),true,5).
-holds(in_hand_f(rob0,l3),true,5).
-holds(in_hand_f(rob0,j8),true,5).
-holds(in_hand_f(rob0,j4),true,5).
-holds(in_hand_f(rob0,j5),true,5).
-holds(in_hand_f(rob0,j6),true,5).
-holds(in_hand_f(rob0,j7),true,5).
-holds(in_hand_f(rob0,j1),true,5).
-holds(in_hand_f(rob0,j2),true,5).
-holds(in_hand_f(rob0,j3),true,5).
-holds(in_hand_c(rob0,b2),true,5).
-holds(in_hand_c(rob0,b3),true,5).
-holds(in_hand_c(rob0,b1),true,5).
-holds(loc_f(rob0,b2t),true,5).
-holds(loc_f(rob0,b4t),true,5).
-holds(loc_f(rob0,b3t),true,5).
-holds(loc_f(rob0,p1a),true,5).
-holds(loc_f(rob0,p3a),true,5).
-holds(loc_f(rob0,p2a),true,5).
-holds(loc_f(rob0,p4a),true,5).
-holds(loc_f(rob0,b2a),true,5).
-holds(loc_f(rob0,p2t),true,5).
-holds(loc_f(rob0,b4a),true,5).
-holds(loc_f(rob0,p1t),true,5).
-holds(loc_f(rob0,b3a),true,5).
-holds(loc_f(rob0,p4t),true,5).
-holds(loc_f(rob0,p3t),true,5).
-holds(loc_f(b3,p3t),true,5).
-holds(loc_f(b3,above_assembly_area),true,5).
-holds(loc_f(b3,p4t),true,5).
-holds(loc_f(b3,b2a),true,5).
-holds(loc_f(b3,p1t),true,5).
-holds(loc_f(b3,b3a),true,5).
-holds(loc_f(b3,p2t),true,5).
-holds(loc_f(b3,b4a),true,5).
-holds(loc_f(b3,p2a),true,5).
-holds(loc_f(b3,p3a),true,5).
-holds(loc_f(b3,p4a),true,5).
-holds(loc_f(b3,b2t),true,5).
-holds(loc_f(b3,b4t),true,5).
-holds(loc_f(b3,p1a),true,5).
-holds(loc_f(b2,p2a),true,5).
-holds(loc_f(b2,p3a),true,5).
-holds(loc_f(b2,b4t),true,5).
-holds(loc_f(b2,p1a),true,5).
-holds(loc_f(b2,b3t),true,5).
-holds(loc_f(b2,p3t),true,5).
-holds(loc_f(b2,p4t),true,5).
-holds(loc_f(b2,p1t),true,5).
-holds(loc_f(b2,b3a),true,5).
-holds(loc_f(b2,p2t),true,5).
-holds(loc_f(b2,b4a),true,5).
-holds(loc_f(b2,b2a),true,5).
-holds(loc_f(b2,above_assembly_area),true,5).
-holds(loc_f(b2,p4a),true,5).
-holds(loc_f(b4,p4a),true,5).
-holds(loc_f(b4,p3a),true,5).
-holds(loc_f(b4,p2a),true,5).
-holds(loc_f(b4,p1a),true,5).
-holds(loc_f(b4,p1t),true,5).
-holds(loc_f(b4,b3a),true,5).
-holds(loc_f(b4,b2a),true,5).
-holds(loc_f(b4,p4t),true,5).
-holds(loc_f(b4,p3t),true,5).
-holds(loc_f(b4,b4a),true,5).
-holds(loc_f(b4,p2t),true,5).
-holds(loc_f(b4,b4t),true,5).
-holds(loc_f(b4,b3t),true,5).
-holds(loc_f(b4,b2t),true,5).
holds(in_hand_f(rob0,l4),true,5).
holds(in_assembly_f(j5),true,5).
holds(in_assembly_f(j4),true,5).
holds(in_assembly_f(j6),true,5).
holds(in_assembly_f(j1),true,5).
holds(in_assembly_f(j3),true,5).
holds(in_assembly_f(j2),true,5).
holds(in_assembly_f(l1),true,5).
holds(in_assembly_f(l3),true,5).
holds(in_assembly_f(l2),true,5).
holds(in_hand_f(rob0,l1),false,5).
holds(in_hand_f(rob0,l2),false,5).
holds(in_hand_f(rob0,l3),false,5).
holds(in_hand_f(rob0,j8),false,5).
holds(in_hand_f(rob0,j4),false,5).
holds(in_hand_f(rob0,j5),false,5).
holds(in_hand_f(rob0,j6),false,5).
holds(in_hand_f(rob0,j7),false,5).
holds(in_hand_f(rob0,j1),false,5).
holds(in_hand_f(rob0,j2),false,5).
holds(in_hand_f(rob0,j3),false,5).
holds(in_hand_c(rob0,b2),false,5).
holds(in_hand_c(rob0,b3),false,5).
holds(in_hand_c(rob0,b1),false,5).
holds(in_assembly_c(b2),true,5).
holds(in_assembly_c(b3),true,5).
holds(in_assembly_c(b1),true,5).
holds(loc_f(rob0,b2t),false,5).
holds(loc_f(rob0,b4t),false,5).
holds(loc_f(rob0,b3t),false,5).
holds(loc_f(rob0,p1a),false,5).
holds(loc_f(rob0,p3a),false,5).
holds(loc_f(rob0,p2a),false,5).
holds(loc_f(rob0,p4a),false,5).
holds(loc_f(rob0,b2a),false,5).
holds(loc_f(rob0,p2t),false,5).
holds(loc_f(rob0,b4a),false,5).
holds(loc_f(rob0,p1t),false,5).
holds(loc_f(rob0,b3a),false,5).
holds(loc_f(rob0,p4t),false,5).
holds(loc_f(rob0,p3t),false,5).
holds(in_hand_c(rob0,b4),true,5).
holds(loc_f(b2,b2t),true,5).
holds(loc_f(b3,b3t),true,5).
holds(loc_c(b3,assembly_area),true,5).
holds(loc_c(b1,assembly_area),true,5).
holds(loc_c(b2,assembly_area),true,5).
holds(loc_f(b3,p3t),false,5).
holds(loc_f(b3,above_assembly_area),false,5).
holds(loc_f(b3,p4t),false,5).
holds(loc_f(b3,b2a),false,5).
holds(loc_f(b3,p1t),false,5).
holds(loc_f(b3,b3a),false,5).
holds(loc_f(b3,p2t),false,5).
holds(loc_f(b3,b4a),false,5).
holds(loc_f(b3,p2a),false,5).
holds(loc_f(b3,p3a),false,5).
holds(loc_f(b3,p4a),false,5).
holds(loc_f(b3,b2t),false,5).
holds(loc_f(b3,b4t),false,5).
holds(loc_f(b3,p1a),false,5).
holds(loc_f(b2,p2a),false,5).
holds(loc_f(b2,p3a),false,5).
holds(loc_f(b2,b4t),false,5).
holds(loc_f(b2,p1a),false,5).
holds(loc_f(b2,b3t),false,5).
holds(loc_f(b2,p3t),false,5).
holds(loc_f(b2,p4t),false,5).
holds(loc_f(b2,p1t),false,5).
holds(loc_f(b2,b3a),false,5).
holds(loc_f(b2,p2t),false,5).
holds(loc_f(b2,b4a),false,5).
holds(loc_f(b2,b2a),false,5).
holds(loc_f(b2,above_assembly_area),false,5).
holds(loc_f(b2,p4a),false,5).
holds(loc_f(b4,p4a),false,5).
holds(loc_f(b4,p3a),false,5).
holds(loc_f(b4,p2a),false,5).
holds(loc_f(b4,p1a),false,5).
holds(loc_f(b4,p1t),false,5).
holds(loc_f(b4,b3a),false,5).
holds(loc_f(b4,b2a),false,5).
holds(loc_f(b4,p4t),false,5).
holds(loc_f(b4,p3t),false,5).
holds(loc_f(b4,b4a),false,5).
holds(loc_f(b4,p2t),false,5).
holds(loc_f(b4,b4t),false,5).
holds(loc_f(b4,b3t),false,5).
holds(loc_f(b4,b2t),false,5).
holds(supported_c(b3),true,5).
holds(supported_c(b2),true,5).
holds(supported_c(b4),true,5).
holds(supported_c(b1),true,5).
holds(supported_f(j2),true,5).
holds(supported_f(l1),true,5).
holds(supported_f(j1),true,5).
holds(supported_f(j8),true,5).
holds(supported_f(l4),true,5).
holds(supported_f(j7),true,5).
holds(supported_f(j4),true,5).
holds(supported_f(l2),true,5).
holds(supported_f(j3),true,5).
holds(supported_f(j6),true,5).
holds(supported_f(l3),true,5).
holds(supported_f(j5),true,5).
-holds(loc_f(rob0,above_assembly_area),false,5).
-holds(loc_c(rob0,assembly_area),false,5).
-holds(loc_f(b4,above_assembly_area),false,5).
-holds(loc_c(b4,assembly_area),false,5).
holds(loc_f(rob0,above_assembly_area),true,5).
holds(loc_c(rob0,assembly_area),true,5).
holds(loc_f(b4,above_assembly_area),true,5).
holds(loc_c(b4,assembly_area),true,5).