%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% AutoGenerated SPARC file
%% Author: MARK ROBSON 2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#const numSteps = 1.
#const startStep = 0.

sorts
#robot = {rob0}.
#beam = {b7,b4,b5,b8}.
#link = {b7l1,b7l2,b7l3,b7l4,b4l1,b4l2,b5l1,b5l2,b8l1,b8l2,b8l3,b8l4}.
#in_m_end = {b4j1,b4j3,b5j1,b5j3}.
#angle_m_end = {dummy_angle_m_end}.
#in_f_end = {b7j1,b7j5,b8j1,b8j5}.
#joint = #in_m_end + #in_f_end + #angle_m_end + #in_f + #thru_m + #angle_f + #thru_f.
#beam_part = #link + #joint.
#pin = {p1,p2,p3,p4,p5}.
#thing = #beam + #pin.
#thing_part = #beam_part + #pin.
#object = #robot + #thing.
#place_c = {input_area,intermediate_area,assembly_area}.
#non_placement_location = {above_input_area,above_intermediate_area,above_assembly_area}.
#approach_location = {b7a,b4a,b5a,b8a,p1a,p2a,p3a,p4a,p5a}.
#prerot_location = {dummy_prerot_location}.
#through_location = {dummy_through_location}.
#target_location = {b7t,b4t,b5t,b8t,p1t,p2t,p3t,p4t,p5t}.
#assembly_location = #approach_location + #target_location + #prerot_location + #through_location.
#near_to_location = {nt_b7i,nt_b4i,nt_b5i,nt_b8i,nt_p1i,nt_p2i,nt_p3i,nt_p4i,nt_p5i,nt_b7t,nt_b4t,nt_b5t,nt_b8t,nt_p1t,nt_p2t,nt_p3t,nt_p4t,nt_p5t,nt_b7a,nt_b4a,nt_b5a,nt_b8a,nt_p1a,nt_p2a,nt_p3a,nt_p4a,nt_p5a}.
#input_location = {b7i,b4i,b5i,b8i,p1i,p2i,p3i,p4i,p5i}.
#place_f = #assembly_location + #near_to_location + #input_location + #non_placement_location.
#fine_res_sort = #place_f + #thing_part.
#coarse_res_sort = #place_c + #thing.
#thru_m = {dummy_thru_m}.
#thru_f = {b7j3,b8j3}.
#angle_f = {b7j2,b7j4,b8j2,b8j4}.
#in_f = {b4j2,b5j2}.
#action = putdown_f(#robot,#thing_part) + move_f(#robot,#place_f) + pick_up_f(#robot,#thing_part) + assemble_f_cap(#robot,#beam_part) + assemble_f_square(#robot,#beam_part) + fasten(#robot,#joint,#joint,#pin) + push(#robot,#beam) + move_local(#robot,#place_f).
#boolean = {true, false}.
#outcome = {true, false, undet}.
#inertial_fluent = in_hand_c(#robot, #thing)+ in_hand_f(#robot, #thing_part)+ loc_c(#object, #place_c)+ loc_f(#object, #place_f)+ in_assembly_c(#beam)+ in_assembly_f(#beam_part)+ supported_c(#beam)+ supported_f(#beam_part)+ fastened_c(#beam, #beam, #pin)+ fastened_f(#beam_part, #beam_part, #pin).
#step = startStep..numSteps.
#fluent = #inertial_fluent.

predicates
next_to_c(#place_c, #place_c).
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
beam_through_loc(#beam, #through_location).
beam_prerotate_loc(#beam, #prerot_location).
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
-near_to(P1,P2):- not near_to(P1,P2).
component(P1,C1):- near_to(C1,C2), component(P1,C2).
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
-fits_into_f(BP1,BP2):- not fits_into_f(BP1,BP2).
-fits_into_c(B1,B2):- not fits_into_c(B1,B2).
holds(fastened_c(B1,B2,P), true, I) :- holds(fastened_f(BP1,BP2,P), true, I), component(B1,BP1), component(B2,BP2).
holds(fastened_f(BP1,BP2,P), true, I) :- holds(fastened_f(BP2,BP1,P), true, I).
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
-occurs(move_f(R,P1), I) :- holds(in_hand_f(R,P), true, I), holds(fastened_c(B1,B2,P), true, I).
-occurs(move_f(R,P2), I) :- holds(loc_f(R,P1), true, I), near_to(P1,P2).
-occurs(move_f(R,P1), I) :- holds(in_hand_c(R,T1), true, I), #target_location(P1).
-occurs(move_f(R,P1), I) :- holds(in_hand_c(R,T1), true, I), #through_location(P1).
-occurs(move_f(R,P1), I) :- holds(in_hand_c(R,T1), true, I), #prerot_location(P1).

holds(in_hand_f(R,T), true, I+1) :- occurs(pick_up_f(R,T), I).
-occurs(pick_up_f(R,P), I) :- holds(loc_f(P,P1), true, I), holds(loc_f(R,P2), true, I), P1!=P2.
-occurs(pick_up_f(R,TP), I) :- holds(loc_f(T1,P1), true, I), holds(loc_f(R,P2), true, I), P1!=P2, component(T1,TP).
-occurs(pick_up_f(R,T1), I) :- holds(in_hand_f(rob0,T2), true, I), #thing_part(T2).
-occurs(pick_up_f(R,T1), I) :- component(B,T1), #beam(B), not #link(T1).

holds(loc_f(R,C), true, I+1) :- occurs(assemble_f_cap(R,BP), I), assem_target_loc(B,C), component(B,BP).
holds(in_assembly_f(BP), true, I+1) :- occurs(assemble_f_cap(R,BP), I).
holds(loc_f(B1,C1), true, I+1) :- occurs(assemble_f_cap(R,BP), I), holds(in_assembly_c(B1), true, I), near_to(C1,C2), holds(loc_f(B1,C2), true, I).
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
holds(loc_f(R,C), true, I+1) :- occurs(assemble_f_square(R,BP), I), beam_prerotate_loc(B,C), component(B,BP), #angle_m_end(BP).
holds(in_assembly_f(BP), true, I+1) :- occurs(assemble_f_square(R,BP), I), not #angle_m_end(BP).
holds(loc_f(B1,C1), true, I+1) :- occurs(assemble_f_square(R,BP), I), holds(in_assembly_c(B1), true, I), near_to(C1,C2), holds(loc_f(B1,C2), true, I).
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

holds(fastened_f(B1,B2,P1), true, I+1) :- occurs(fasten(R,B1,B2,P1), I).
holds(loc_f(R,C1), true, I+1) :- occurs(fasten(R,B1,B2,P1), I), assem_target_loc(P1,C1).
-occurs(fasten(R,BP1,BP2,P1), I) :- not holds(in_assembly_f(BP1), true, I).
-occurs(fasten(R,BP1,BP2,P1), I) :- not holds(in_assembly_f(BP2), true, I).
-occurs(fasten(R,B1,B2,P1), I) :- not holds(in_hand_f(R,P1), true, I).
-occurs(fasten(R,BP1,BP2,P1), I) :- holds(loc_f(B1,C1), true, I), assem_target_loc(B1,C2), component(B1,BP1), C1!=C2.
-occurs(fasten(R,BP1,BP2,P1), I) :- holds(loc_f(B1,C1), true, I), assem_target_loc(B1,C2), component(B1,BP2), C1!=C2.
-occurs(fasten(R,BP1,BP2,P1), I) :- holds(loc_f(R,C1), true, I), assem_approach_loc(P1,C2), C1!=C2.
-occurs(fasten(R,BP1,BP2,P1), I) :- not fits_into_f(BP1,BP2).
-occurs(fasten(R,BP1,BP2,P1), I) :- holds(fastened_f(BP1,BP2,P2), true, I).

holds(loc_f(B,C), true, I+1) :- occurs(push(R,B), I), assem_target_loc(B,C).
holds(loc_f(R,C), true, I+1) :- occurs(push(R,B), I), assem_target_loc(B,C).
-occurs(push(R,B), I) :- not holds(in_assembly_c(B), true, I).
-occurs(push(R,B), I) :- holds(in_hand_c(R,T), true, I), #thing(T).
-occurs(push(R,B), I) :- holds(loc_f(R,C1), true, I), not near_to(C1,C2), assem_target_loc(B,C2).
-occurs(push(R,B), I) :- holds(loc_f(R,C1), true, I), holds(loc_f(B,C2), true, I), C1!=C2.
-occurs(push(R,B), I) :- holds(loc_f(R,C1), true, I), not #near_to_location(C1).

holds(loc_f(R,C1), true, I+1) :- occurs(move_local(R,C1), I).
-occurs(move_local(R,C2), I) :- holds(loc_f(R,C1), true, I), not near_to(C1,C2).
-occurs(move_local(R,P2), I) :- holds(loc_f(R,P1), true, I), not #near_to_location(P1).
-occurs(move_local(R,P1), I) :- holds(loc_f(R,P2), true, I), P1=P2.

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
goal(I) :- .

% domain setup
% robot location coarse
holds(loc_c(rob0,intermediate_area),true,0).
% assembly relations
holds(in_assembly_c(b7),true,0).
% beam and pin locations coarse
holds(loc_c(p1,input_area),true,0).
holds(loc_c(p2,input_area),true,0).
holds(loc_c(p3,input_area),true,0).
holds(loc_c(p4,input_area),true,0).
holds(loc_c(b7,assembly_area),true,0).
holds(loc_c(b4,input_area),true,0).
holds(loc_c(b5,input_area),true,0).
holds(loc_c(b8,input_area),true,0).
% assert robots hand is empty at timestep 0
holds(in_hand_c(rob0,b7),false,0).
holds(in_hand_c(rob0,b4),false,0).
holds(in_hand_c(rob0,b5),false,0).
holds(in_hand_c(rob0,b8),false,0).
holds(in_hand_c(rob0,p1),false,0).
holds(in_hand_c(rob0,p2),false,0).
holds(in_hand_c(rob0,p3),false,0).
holds(in_hand_c(rob0,p4),false,0).
% robot location fine
holds(loc_f(rob0,above_intermediate_area),true,0).
% beam and pin locations fine
holds(loc_f(b7,b7t),true,0).
holds(loc_f(b4,b4i),true,0).
holds(loc_f(b5,b5i),true,0).
holds(loc_f(b8,b8i),true,0).
holds(loc_f(p1,p1i),true,0).
holds(loc_f(p2,p2i),true,0).
holds(loc_f(p3,p3i),true,0).
holds(in_hand_f(rob0,b4j1),false,0).
holds(in_hand_f(rob0,b4j2),false,0).
holds(in_hand_f(rob0,b4j3),false,0).
holds(in_hand_f(rob0,b4l1),false,0).
holds(in_hand_f(rob0,b4l2),false,0).
holds(in_hand_f(rob0,b5j1),false,0).
holds(in_hand_f(rob0,b5j2),false,0).
holds(in_hand_f(rob0,b5j3),false,0).
holds(in_hand_f(rob0,b5l1),false,0).
holds(in_hand_f(rob0,b5l2),false,0).
holds(in_hand_f(rob0,b8j1),false,0).
holds(in_hand_f(rob0,b8j2),false,0).
holds(in_hand_f(rob0,b8j3),false,0).
holds(in_hand_f(rob0,b8j4),false,0).
holds(in_hand_f(rob0,b8j5),false,0).
holds(in_hand_f(rob0,b8l1),false,0).
holds(in_hand_f(rob0,b8l2),false,0).
holds(in_hand_f(rob0,b8l3),false,0).
holds(in_hand_f(rob0,b8l4),false,0).
holds(in_hand_f(rob0,b7j1),false,0).
holds(in_hand_f(rob0,b7j2),false,0).
holds(in_hand_f(rob0,b7j3),false,0).
holds(in_hand_f(rob0,b7j4),false,0).
holds(in_hand_f(rob0,b7j5),false,0).
holds(in_hand_f(rob0,b7l1),false,0).
holds(in_hand_f(rob0,b7l2),false,0).
holds(in_hand_f(rob0,b7l3),false,0).
holds(in_hand_f(rob0,b7l4),false,0).
holds(in_hand_f(rob0,p1),false,0).
holds(in_hand_f(rob0,p2),false,0).
holds(in_hand_f(rob0,p3),false,0).
holds(in_hand_f(rob0,p4),false,0).
holds(loc_f(p4,p4i),true,0).
component(p1,p1).
component(p2,p2).
component(p3,p3).
component(p4,p4).
component(p5,p5).
fits_into_f(b4j1,b7j1).
fits_into_f(b5j1,b7j5).
fits_into_f(b4j3,b8j1).
fits_into_f(b5j3,b8j5).
connected_to(b7j1,b7l1).
connected_to(b7l1,b7j2).
connected_to(b7j2,b7l2).
connected_to(b7l2,b7j3).
connected_to(b7j3,b7l3).
connected_to(b7l3,b7j4).
connected_to(b7j4,b7l4).
connected_to(b7l4,b7j5).
component(b7,b7j1).
component(b7,b7l1).
component(b7,b7j2).
component(b7,b7l2).
component(b7,b7j3).
component(b7,b7l3).
component(b7,b7j4).
component(b7,b7l4).
component(b7,b7j5).
connected_to(b4j1,b4l1).
connected_to(b4l1,b4j2).
connected_to(b4j2,b4l2).
connected_to(b4l2,b4j3).
component(b4,b4j1).
component(b4,b4l1).
component(b4,b4j2).
component(b4,b4l2).
component(b4,b4j3).
connected_to(b5j1,b5l1).
connected_to(b5l1,b5j2).
connected_to(b5j2,b5l2).
connected_to(b5l2,b5j3).
component(b5,b5j1).
component(b5,b5l1).
component(b5,b5j2).
component(b5,b5l2).
component(b5,b5j3).
connected_to(b8j1,b8l1).
connected_to(b8l1,b8j2).
connected_to(b8j2,b8l2).
connected_to(b8l2,b8j3).
connected_to(b8j3,b8l3).
connected_to(b8l3,b8j4).
connected_to(b8j4,b8l4).
connected_to(b8l4,b8j5).
component(b8,b8j1).
component(b8,b8l1).
component(b8,b8j2).
component(b8,b8l2).
component(b8,b8j3).
component(b8,b8l3).
component(b8,b8j4).
component(b8,b8l4).
component(b8,b8j5).
assem_target_loc(b7,b7t).
assem_target_loc(b4,b4t).
assem_target_loc(b5,b5t).
assem_target_loc(b8,b8t).
assem_target_loc(p1,p1t).
assem_target_loc(p2,p2t).
assem_target_loc(p3,p3t).
assem_target_loc(p4,p4t).
assem_target_loc(p5,p5t).
assem_approach_loc(b7,b7a).
assem_approach_loc(b4,b4a).
assem_approach_loc(b5,b5a).
assem_approach_loc(b8,b8a).
assem_approach_loc(p1,p1a).
assem_approach_loc(p2,p2a).
assem_approach_loc(p3,p3a).
assem_approach_loc(p4,p4a).
assem_approach_loc(p5,p5a).
near_to(nt_b7i,b7i).
near_to(nt_b4i,b4i).
near_to(nt_b5i,b5i).
near_to(nt_b8i,b8i).
near_to(nt_p1i,p1i).
near_to(nt_p2i,p2i).
near_to(nt_p3i,p3i).
near_to(nt_p4i,p4i).
near_to(nt_p5i,p5i).
near_to(nt_b7t,b7t).
near_to(nt_b4t,b4t).
near_to(nt_b5t,b5t).
near_to(nt_b8t,b8t).
near_to(nt_p1t,p1t).
near_to(nt_p2t,p2t).
near_to(nt_p3t,p3t).
near_to(nt_p4t,p4t).
near_to(nt_p5t,p5t).
near_to(nt_b7a,b7a).
near_to(nt_b4a,b4a).
near_to(nt_b5a,b5a).
near_to(nt_b8a,b8a).
near_to(nt_p1a,p1a).
near_to(nt_p2a,p2a).
near_to(nt_p3a,p3a).
near_to(nt_p4a,p4a).
near_to(nt_p5a,p5a).
component(input_area,b7i).
component(input_area,b4i).
component(input_area,b5i).
component(input_area,b8i).
component(input_area,p1i).
component(input_area,p2i).
component(input_area,p3i).
component(input_area,p4i).
component(input_area,p5i).
component(assembly_area,b7t).
component(assembly_area,b4t).
component(assembly_area,b5t).
component(assembly_area,b8t).
component(assembly_area,p1t).
component(assembly_area,p2t).
component(assembly_area,p3t).
component(assembly_area,p4t).
component(assembly_area,p5t).
component(assembly_area,b7a).
component(assembly_area,b4a).
component(assembly_area,b5a).
component(assembly_area,b8a).
component(assembly_area,p1a).
component(assembly_area,p2a).
component(assembly_area,p3a).
component(assembly_area,p4a).
component(assembly_area,p5a).
next_to_f(b7t,b7a).
next_to_f(b4t,b4a).
next_to_f(b5t,b5a).
next_to_f(b8t,b8a).
next_to_f(p1t,p1a).
next_to_f(p2t,p2a).
next_to_f(p3t,p3a).
next_to_f(p4t,p4a).
next_to_f(p5t,p5a).
next_to_f(above_assembly_area,b7a).
next_to_f(above_assembly_area,b4a).
next_to_f(above_assembly_area,b5a).
next_to_f(above_assembly_area,b8a).
next_to_f(above_assembly_area,p1a).
next_to_f(above_assembly_area,p2a).
next_to_f(above_assembly_area,p3a).
next_to_f(above_assembly_area,p4a).
next_to_f(above_assembly_area,p5a).
next_to_f(above_assembly_area,b7t).
next_to_f(above_assembly_area,b4t).
next_to_f(above_assembly_area,b5t).
next_to_f(above_assembly_area,b8t).
next_to_f(above_assembly_area,p1t).
next_to_f(above_assembly_area,p2t).
next_to_f(above_assembly_area,p3t).
next_to_f(above_assembly_area,p4t).
next_to_f(above_assembly_area,p5t).
next_to_f(above_assembly_area,nt_b7t).
next_to_f(above_assembly_area,nt_b4t).
next_to_f(above_assembly_area,nt_b5t).
next_to_f(above_assembly_area,nt_b8t).
next_to_f(above_assembly_area,nt_p1t).
next_to_f(above_assembly_area,nt_p2t).
next_to_f(above_assembly_area,nt_p3t).
next_to_f(above_assembly_area,nt_p4t).
next_to_f(above_assembly_area,nt_p5t).
next_to_f(above_assembly_area,nt_b7a).
next_to_f(above_assembly_area,nt_b4a).
next_to_f(above_assembly_area,nt_b5a).
next_to_f(above_assembly_area,nt_b8a).
next_to_f(above_assembly_area,nt_p1a).
next_to_f(above_assembly_area,nt_p2a).
next_to_f(above_assembly_area,nt_p3a).
next_to_f(above_assembly_area,nt_p4a).
next_to_f(above_assembly_area,nt_p5a).
next_to_f(above_input_area,nt_b7i).
next_to_f(above_input_area,nt_b4i).
next_to_f(above_input_area,nt_b5i).
next_to_f(above_input_area,nt_b8i).
next_to_f(above_input_area,nt_p1i).
next_to_f(above_input_area,nt_p2i).
next_to_f(above_input_area,nt_p3i).
next_to_f(above_input_area,nt_p4i).
next_to_f(above_input_area,nt_p5i).
next_to_f(above_input_area,b7i).
next_to_f(above_input_area,b4i).
next_to_f(above_input_area,b5i).
next_to_f(above_input_area,b8i).
next_to_f(above_input_area,p1i).
next_to_f(above_input_area,p2i).
next_to_f(above_input_area,p3i).
next_to_f(above_input_area,p4i).
next_to_f(above_input_area,p5i).
% coarse next_to location mapping
next_to_c(input_area,intermediate_area).
next_to_c(assembly_area,intermediate_area).
% next_to_f location map
next_to_f(above_input_area,above_intermediate_area).
next_to_f(above_assembly_area,above_intermediate_area).
component(input_area,above_input_area).
component(intermediate_area,above_intermediate_area).
component(assembly_area,above_assembly_area).