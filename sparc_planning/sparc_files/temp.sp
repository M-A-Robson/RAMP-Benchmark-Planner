%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% AutoGenerated SPARC file
%% Author: MARK ROBSON 2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#const numSteps = 10.
#const startStep = 0.

sorts
#robot = {rob0}.
#beam = {b7,b4,b5,b8}.
#pin = {p1,p2,p3,p4,p5}.
#thing = #beam + #pin.
#object = #robot + #thing.
#place_c = {input_area,intermediate_area,assembly_area}.
#action = putdown(#robot,#thing) + move(#robot,#place_c) + pick_up(#robot,#thing) + assemble(#robot,#beam) + fasten(#robot,#beam,#beam,#pin) + push(#robot,#beam).
#boolean = {true, false}.
#outcome = {true, false, undet}.
#inertial_fluent = in_hand_c(#robot, #thing)+ loc_c(#object, #place_c)+ in_assembly_c(#beam)+ supported_c(#beam)+ fastened_c(#beam, #beam, #pin)+ misaligned_c(#beam)+ can_fasten_c(#beam, #beam).
#step = startStep..numSteps.
#fluent = #inertial_fluent.

predicates
next_to_c(#place_c, #place_c).
fits_into_c(#beam, #beam).
fits_through_c(#beam, #beam).
is_capped_by(#beam, #beam, #beam).
base(#beam).
holds(#fluent, #boolean, #step).
occurs(#action, #step).
success().
goal(#step).
something_happened(#step).

rules
-base(B2):- base(B1), B1!=B2.
next_to_c(P1,P2):- next_to_c(P2,P1).
holds(fastened_c(B1,B2,P1), true, I) :- holds(fastened_c(B2,B1,P1), true, I).
-next_to_c(P1,P2):- not next_to_c(P1,P2).
holds(loc_c(T,P1), false, I) :- holds(loc_c(T,P2), true, I), P1!=P2.
holds(loc_c(T1,P1), true, I) :- holds(loc_c(R,P1), true, I), holds(in_hand_c(R,T2), true, I), T1=T2.
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
-fits_into_c(B1,B2):- not fits_into_c(B1,B2).

holds(in_hand_c(R,T), false, I+1) :- occurs(putdown(R,T), I).
-occurs(putdown(R,T), I) :- not holds(in_hand_c(R,T), true, I).

holds(loc_c(R,P), true, I+1) :- occurs(move(R,P), I).
-occurs(move(R,P1), I) :- holds(loc_c(R,P2), true, I), P1=P2.
-occurs(move(R,P1), I) :- holds(loc_c(R,P2), true, I), not next_to_c(P1,P2).
-occurs(move(R,P1), I) :- holds(in_hand_c(R,B1), true, I), holds(in_assembly_c(B1), true, I).
-occurs(move(R,P1), I) :- holds(in_hand_c(R,P), true, I), holds(fastened_c(B1,B2,P), true, I).

holds(in_hand_c(R,T), true, I+1) :- occurs(pick_up(R,T), I).
-occurs(pick_up(R,T1), I) :- holds(loc_c(T1,P1), true, I), holds(loc_c(R,P2), true, I), P1!=P2.
-occurs(pick_up(R,T1), I) :- holds(in_hand_c(R,T2), true, I), #thing(T2).

holds(in_assembly_c(B), true, I+1) :- occurs(assemble(R,B), I).
holds(misaligned_c(B2), true, I+1) :- occurs(assemble(R,B1), I), holds(in_assembly_c(B2), true, I), not base(B2), B1!=B2.
holds(can_fasten_c(B1,B2), true, I+1) :- occurs(assemble(R,B1), I), holds(in_assembly_c(B2), true, I), fits_into_c(B1,B2), B1!=B2.
holds(can_fasten_c(B1,B2), true, I+1) :- occurs(assemble(R,B1), I), holds(in_assembly_c(B2), true, I), fits_through_c(B1,B2), B1!=B2.
-occurs(assemble(R,B), I) :- not holds(in_hand_c(R,B), true, I).
-occurs(assemble(R,B1), I) :- holds(in_assembly_c(B2), true, I), holds(in_assembly_c(B3), true, I), is_capped_by(B1,B2,B3), B2!=B3.
-occurs(assemble(R,B1), I) :- holds(in_assembly_c(B2), true, I), fits_through_c(B2,B1).
-occurs(assemble(R,B1), I) :- not holds(in_assembly_c(B2), true, I), holds(in_assembly_c(B3), true, I), is_capped_by(B2,B1,B3), B2!=B3.
-occurs(assemble(R,B), I) :- holds(loc_c(R,P), true, I), P!=assembly_area.
-occurs(assemble(R,B), I) :- holds(in_assembly_c(B), true, I).
-occurs(assemble(R,B), I) :- not holds(supported_c(B), true, I).
-occurs(assemble(R,B1), I) :- holds(in_assembly_c(B2), true, I), holds(misaligned_c(B2), true, I).
-occurs(assemble(R,B1), I) :- holds(can_fasten_c(B2,B3), true, I), B1!=B2, B1!=B3.

holds(fastened_c(B1,B2,P1), true, I+1) :- occurs(fasten(R,B1,B2,P1), I).
holds(can_fasten_c(B1,B2), false, I+1) :- occurs(fasten(R,B1,B2,P1), I).
-occurs(fasten(R,B1,B2,P1), I) :- not holds(in_assembly_c(B1), true, I).
-occurs(fasten(R,B1,B2,P1), I) :- not holds(in_assembly_c(B2), true, I).
-occurs(fasten(R,B1,B2,P1), I) :- not holds(in_hand_c(R,P1), true, I).
-occurs(fasten(R,B1,B2,P1), I) :- not fits_into_c(B1,B2).
-occurs(fasten(R,B1,B2,P1), I) :- holds(fastened_c(B1,B2,P2), true, I).
-occurs(fasten(R,B1,B2,P1), I) :- holds(loc_c(B1,L1), true, I), holds(loc_c(R,L2), true, I), L1!=L2.
-occurs(fasten(R,B1,B2,P1), I) :- holds(misaligned_c(B1), true, I).
-occurs(fasten(R,B1,B2,P1), I) :- holds(misaligned_c(B2), true, I).

holds(misaligned_c(B1), false, I+1) :- occurs(push(R,B1), I).
-occurs(push(R,B1), I) :- holds(loc_c(B1,L1), true, I), holds(loc_c(R,L2), true, I), L1!=L2.
-occurs(push(R,B1), I) :- not holds(in_assembly_c(B1), true, I).
-occurs(push(R,B1), I) :- holds(in_hand_c(R,T), true, I), #thing(T).

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
goal(I) :- holds(in_assembly_c(b4), true, I) , holds(fastened_c(b7,b4,p1), true, I) , holds(in_assembly_c(b5), true, I) , holds(fastened_c(b7,b5,p2), true, I) , holds(in_assembly_c(b8), true, I) , holds(fastened_c(b5,b8,p3), true, I) , holds(fastened_c(b4,b8,p4), true, I).

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
fits_into_c(b4,b7).
fits_into_c(b5,b7).
fits_into_c(b4,b8).
fits_into_c(b5,b8).
base(b7).
% coarse next_to location mapping
next_to_c(input_area,intermediate_area).
next_to_c(assembly_area,intermediate_area).
base(b7).
