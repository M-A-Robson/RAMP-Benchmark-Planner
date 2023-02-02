%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% AutoGenerated SPARC file
%% Author: MARK ROBSON 2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#const numSteps = 25.

sorts
#robot = {rob0}.
#beam = {b1,b2,b3,b4}.
#link = {L1,L2,L3,L4}.
#in_m_end = {J3,J4,J7,J8}.
#in_f_end = {J1,J2,J5,J6}.
#end_joint = #in_m_end + #in_f_end.
#beam_part = #link + #end_joint.
#pin = {p1,p2,p3,p4}.
#thing = #beam + #pin.
#thing_part = #beam_part + #pin.
#object = #robot + #thing.
#place = {input_area,intermediate_area,assembly_area}.
#non_placement_location = {above_input,above_intermediate,above_assembly}.
#approach_location = {b1a,b2a,b3a,b4a,p1a,p2a,p3a,p4a}.
#prerot_location = {}.
#through_location = {}.
#target_location = {b1t,b2t,b3t,b4t,p1t,p2t,p3t,p4t}.
#near_to_location = {nt_b1a,nt_b2a,nt_b3a,nt_b4a,nt_p1a,nt_p2a,nt_p3a,nt_p4a,nt_b1t,nt_b2t,nt_b3t,nt_b4t,nt_p1t,nt_p2t,nt_p3t,nt_p4t,nt_above_input,nt_above_intermediate,nt_above_assembly}.
#input_locations = {b2i,b3i,b4i,p1i,p2i,p3i,p4i}.
#place_f = #approach_location + #target_location + #non_placement_location + #near_to_location + #input_locations.
#fine_res_sort = #place_f.
#coarse_res_sort = #place.
#action = putdown_f(#robot,#thing_part) + move_f(#robot,#place_f) + pick_up_f(#robot,#thing_part) + assemble_f_cap(#robot,#beam_part) + assemble_f_square(#robot,#beam_part) + fasten(#robot,#beam_part,#beam_part,#pin) + push(#robot,#beam) + move_local(#robot,#place_f).
#boolean = {true, false}.
#outcome = {true, false, undet}.
#inertial_fluent = in_hand_c(#robot, #thing)+ in_hand_f(#robot, #thing_part)+ loc_c(#object, #place)+ loc_f(#object, #place_f)+ in_assembly_c(#beam)+ in_assembly_f(#beam_part)+ supported_c(#beam)+ supported_f(#beam_part)+ fastened_c(#beam, #beam, #pin)+ fastened_f(#beam_part, #beam_part, #pin).
#step = 0..numSteps.
#fluent = #inertial_fluent.

predicates
next_to_c(#place, #place).
next_to_f(#place_f, #place_f).
component(#coarse_res_sort, #fine_res_sort).
near_to(#near_to_location, #place_f).
fits_into_c(#beam, #beam).
fits_into_f(#beam_part, #beam_part).
fits_through_c(#beam, #beam).
fits_through_f(#beam_part, #beam_part).
is_capped_by(#beam, #beam, #beam).
connected_to(#beam_part, #beam_part).
between(#beam_part, #beam_part, #beam_part).
assem_approach_loc(#thing, #approach_location).
assem_target_loc(#thing, #target_location).
assembly_loc(#thing, #place_f).
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
next_to_c(P1,P2):- next_to_f(C1,C2), component(P1,C1), component(P2,C2).
holds(loc_c(T,P), true, I) :- holds(loc_f(T,C), true, I), component(P,C).
holds(loc_f(T1,P1), true, I) :- holds(loc_f(R,P1), true, I), holds(in_hand_c(R,T2), true, I), T1=T2.
-component(C1,P1):- #thing(P1), not #thing_part(C1).
holds(in_hand_c(R,T), true, I) :- holds(in_hand_f(R,TP), true, I), component(TP,T).
holds(supported_c(B1), true, I) :- holds(supported_f(P1), true, I), component(B1,P1).
holds(supported_f(P1), true, I) :- holds(supported_c(B1), true, I), component(B1,P1).
holds(in_assembly_c(B1), true, I) :- holds(in_assembly_f(P1), true, I), component(B1,P1).
holds(in_assembly_f(P1), true, I) :- holds(in_assembly_c(B1), true, I), component(B1,P1).
assembly_loc(T,C):- assem_approach_loc(T,C).
assembly_loc(T,C):- assem_target_loc(T,C).
holds(in_assembly_c(B), true, I) :- holds(loc_f(B,C), true, I), assem_target_loc(B,C).
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
holds(fastened_c(B1,B2,P), true, I) :- holds(fastened_f(BP1,BP2,P), true, I), component(BP1,B1), component(BP2,B2).
holds(fastened_f(BP1,BP2,P), true, I) :- holds(fastened_f(BP2,BP1,P), true, I).
connected_to(BP1,BP2):- connected_to(BP2,BP1).
-connected_to(BP1,BP2):- not connected_to(BP1,BP2).
between(BP1,BP2,BP3):- connected_to(BP1,BP2), connected_to(BP1,BP3).
between(BP1,BP2,BP3):- connected_to(BP1,BP2), between(BP4,BP1,BP3).
between(BP1,BP2,BP3):- between(BP4,BP1,BP3), between(BP5,BP1,BP3).
between(BP1,BP2,BP3):- between(BP1,BP3,BP2).
fits_into_c(B1,B2):- fits_into_f(D1,D2), component(D1,B1), component(D2,B2).
fits_through_c(B1,B2):- fits_through_f(D1,D2), component(D1,B1), component(D2,B2).
holds(in_hand_f(R,T), false, I+1) :- occurs(putdown_f(R,T), I).
-occurs(putdown_f(R,T), I) :- not holds(in_hand_f(R,T), true, I).
holds(loc_f(R,P), true, I+1) :- occurs(move_f(R,P), I).
-occurs(move_f(R,P1), I) :- holds(loc_f(R,P2), true, I), P1=P2.
-occurs(move_f(R,P1), I) :- holds(loc_f(R,P2), true, I), not next_to_f(P1,P2).
-occurs(move_f(R,P1), I) :- holds(in_hand_f(R,BP), true, I), holds(in_assembly_f(BP), true, I).
-occurs(move_f(R,P1), I) :- holds(in_hand_f(R,P), true, I), holds(fastened_c(B1,B2,P), true, I).
-occurs(move_f(R,P2), I) :- holds(loc_f(R,P1), true, I), near_to(P1,P2).
holds(in_hand_f(R,T), true, I+1) :- occurs(pick_up_f(R,T), I).
-occurs(pick_up_f(R,T1), I) :- holds(loc_f(T,P1), true, I), holds(loc_f(R,P2), true, I), P1!=P2, component(T,T1).
-occurs(pick_up_f(R,T1), I) :- holds(in_hand_f(rob0,T2), true, I), #thing_part(T2).
holds(loc_f(R,C), true, I+1) :- occurs(assemble_f_cap(R,BP), I), assem_target_loc(B,C), component(B,BP).
holds(loc_f(B1,C1), true, I+1) :- occurs(assemble_f_cap(R,BP), I), component(B2,BP), holds(in_assembly_c(B1), true, I), near_to(C1,C2), assem_target_loc(B1,C2).
-occurs(assemble_f_cap(R,BP), I) :- not holds(in_assembly_c(B2), true, I), not holds(in_assembly_c(B3), true, I), is_capped_by(B1,B2,B3), B2!=B3, component(B1,BP).
-occurs(assemble_f_cap(R,BP), I) :- component(B1,BP), holds(loc_f(R,C1), true, I), assem_approach_loc(B1,C2), C1!=C2.
-occurs(assemble_f_cap(R,BP), I) :- not holds(in_hand_c(R,B), true, I), component(B,BP).
-occurs(assemble_f_cap(R,BP), I) :- holds(in_hand_f(R,,BP), true, I).
-occurs(assemble_f_cap(R,BP), I) :- holds(in_assembly_c(B2), true, I), holds(in_assembly_c(B3), true, I), is_capped_by(B1,B2,B3), B2!=B3, component(B1,BP).
-occurs(assemble_f_cap(R,BP), I) :- holds(in_assembly_f(BP2), true, I), fits_through_f(BP2,BP).
-occurs(assemble_f_cap(R,BP), I) :- not holds(in_assembly_c(B2), true, I), holds(in_assembly_c(B3), true, I), is_capped_by(B2,B1,B3), B2!=B3, component(B1,BP).
-occurs(assemble_f_cap(R,BP), I) :- holds(in_assembly_f(BP), true, I).
-occurs(assemble_f_cap(R,BP), I) :- not holds(supported_f(BP), true, I).
-occurs(assemble_f_cap(R,BP), I) :- holds(in_assembly_c(B2), true, I), #beam(B2), holds(loc_f(B2,C1), true, I), assem_target_loc(B2,C2), C1!=C2.
holds(loc_f(R,C), true, I+1) :- occurs(assemble_f_square(R,BP), I), assem_target_loc(B,C), component(B,BP), not #angle_m_end(BP).
holds(loc_f(R,C), true, I+1) :- occurs(assemble_f_square(R,BP), I), #prerot_location(B,C), component(B,BP), #angle_m_end(BP).
holds(loc_f(B1,C1), true, I+1) :- occurs(assemble_f_square(R,BP), I), component(B2,BP), holds(in_assembly_c(B1), true, I), near_to(C1,C2), assem_target_loc(B1,C2).
-occurs(assemble_f_square(R,BP), I) :- holds(in_assembly_c(B2), true, I), holds(in_assembly_c(B3), true, I), is_capped_by(B1,B2,B3), B2!=B3, component(B1,BP).
-occurs(assemble_f_square(R,BP), I) :- component(B1,BP), holds(loc_f(R,C1), true, I), assem_approach_loc(B1,C2), C1!=C2.
-occurs(assemble_f_square(R,BP), I) :- not holds(in_hand_c(R,B), true, I), component(B,BP).
-occurs(assemble_f_square(R,BP), I) :- holds(in_hand_f(R,,BP), true, I).
-occurs(assemble_f_square(R,BP), I) :- holds(in_assembly_c(B2), true, I), holds(in_assembly_c(B3), true, I), is_capped_by(B1,B2,B3), B2!=B3, component(B1,BP).
-occurs(assemble_f_square(R,BP), I) :- holds(in_assembly_f(BP2), true, I), fits_through_f(BP2,BP).
-occurs(assemble_f_square(R,BP), I) :- not holds(in_assembly_c(B2), true, I), holds(in_assembly_c(B3), true, I), is_capped_by(B2,B1,B3), B2!=B3, component(B1,BP).
-occurs(assemble_f_square(R,BP), I) :- holds(in_assembly_f(BP), true, I).
-occurs(assemble_f_square(R,BP), I) :- not holds(supported_f(BP), true, I).
-occurs(assemble_f_square(R,BP), I) :- holds(in_assembly_c(B2), true, I), #beam(B2), holds(loc_f(B2,C1), true, I), assem_target_loc(B2,C2), C1!=C2.
holds(fastened_f(B1,B2,P1), true, I+1) :- occurs(fasten(R,B1,B2,P1), I).
holds(loc_f(R,C1), true, I+1) :- occurs(fasten(R,B1,B2,P1), I), assem_target_loc(P1,C1).
-occurs(fasten(R,BP1,BP2,P1), I) :- not holds(in_assembly_f(BP1), true, I).
-occurs(fasten(R,BP1,BP2,P1), I) :- not holds(in_assembly_f(BP2), true, I).
-occurs(fasten(R,B1,B2,P1), I) :- not holds(in_hand_f(R,P1), true, I).
-occurs(fasten(R,BP1,BP2,P1), I) :- holds(loc_f(B1,C1), true, I), assem_target_loc(B1,C2), component(B1,BP1), C1!=C2.
-occurs(fasten(R,BP1,BP2,P1), I) :- holds(loc_f(B1,C1), true, I), assem_target_loc(B1,C2), component(B1,BP2), C1!=C2.
-occurs(fasten(R,BP1,BP2,P1), I) :- holds(loc_f(R,C1), true, I), assem_approach_loc(P1,C2), C1!=C2.
holds(loc_f(B,C), true, I+1) :- occurs(push(R,B), I), assem_target_loc(B,C).
-occurs(push(R,B), I) :- not holds(in_assembly_c(B), true, I).
-occurs(push(R,B), I) :- holds(in_hand_c(R,T), true, I), #thing(T).
-occurs(push(R,B), I) :- holds(loc_f(R,C1), true, I), not near_to(C1,C2), assem_target_loc(B,C2).
holds(loc_f(R,C1), true, I+1) :- occurs(move_local(R,C1), I).
-occurs(move_local(R,C2), I) :- not holds(loc_f(R,C1), true, I), near_to(C1,C2).
-holds(F, V2, I) :- holds(F, V1, I), V1!=V2.
holds(F, Y, I+1) :- #inertial_fluent(F), holds(F, Y, I), not -holds(F, Y, I+1), I < numSteps.
-occurs(A,I) :- not occurs(A,I).
success :- goal(I), I <= numSteps.
:- not success.
occurs(A, I) | -occurs(A, I) :- not goal(I).
-occurs(A2, I) :- occurs(A1, I), A1 != A2.
something_happened(I) :- occurs(A, I).
:- not goal(I), not something_happened(I).
goal(I) :- holds(in_assembly_c(b2), true, I).
% beam locations
holds(in_assembly(b1),true,0).
holds(loc_f(b1,b1t),true,0).
holds(loc_f(b2,b2i),true,0).
holds(loc_f(b3,b3i),true,0).
holds(loc_f(b4,b4i),true,0).
% beam components and connections
connected_to(J1,L1).
connected_to(L1,J2).
component(J1,b1).
component(L1,b1).
component(J2,b1).
connected_to(J3,L2).
connected_to(L2,J4).
component(J3,b2).
component(L2,b2).
component(J4,b2).
connected_to(J5,L3).
connected_to(L3,J6).
component(J5,b3).
component(L3,b3).
component(J6,b3).
connected_to(J7,L4).
connected_to(L4,J8).
component(J7,b4).
component(L4,b4).
component(J8,b4).
% robot location
holds(loc_f(rob0,above_input_area),true,0).
% next_to_f location map
next_to_f(above_input_area,b2i).
next_to_f(above_input_area,b3i).
next_to_f(above_input_area,b4i).
next_to_f(above_input_area,p1i).
next_to_f(above_input_area,p2i).
next_to_f(above_input_area,p3i).
next_to_f(above_input_area,p4i).
next_to_f(above_input_area,above_intermediate_area).
next_to_f(above_assembly_area,above_intermediate_area).
next_to_f(above_assembly_area,b2a).
next_to_f(above_assembly_area,b3a).
next_to_f(above_assembly_area,b4a).
next_to_f(b2t,b2a).
next_to_f(b3t,b3a).
next_to_f(b4t,b4a).
% beam to beam connections
fits_into_f(J3, J1).
fits_into_f(b5, J2).
fits_into_f(b6, J8).
fits_into_f(J4, J7).
holds(in_hand_f(rob0,L1),false,0).
holds(in_hand_f(rob0,L2),false,0).
holds(in_hand_f(rob0,L3),false,0).
holds(in_hand_f(rob0,L4),false,0).
holds(in_hand_f(rob0,J3),false,0).
holds(in_hand_f(rob0,J4),false,0).
holds(in_hand_f(rob0,J7),false,0).
holds(in_hand_f(rob0,J8),false,0).
holds(in_hand_f(rob0,J1),false,0).
holds(in_hand_f(rob0,J2),false,0).
holds(in_hand_f(rob0,J5),false,0).
holds(in_hand_f(rob0,J6),false,0).
holds(in_hand_f(rob0,p1),false,0).
holds(in_hand_f(rob0,p2),false,0).
holds(in_hand_f(rob0,p3),false,0).
holds(in_hand_f(rob0,p4),false,0).

display
occurs.
