%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% AutoGenerated SPARC file
%% Author: MARK ROBSON 2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#const numSteps = 18.

sorts
#robot = {rob0}.
#beam = {b1,b2,b3,b4}.
#pin = {p1,p2,p3,p4}.
#thing = #beam + #pin.
#object = #robot + #thing.
#place = {input_area,intermediate_area,assembly_area}.
#action = putdown(#robot,#thing) + move(#robot,#place) + pick_up(#robot,#thing) + assemble(#robot,#beam) + fasten(#robot,#beam,#beam,#pin).
#boolean = {true, false}.
#outcome = {true, false, undet}.
#inertial_fluent = in_hand(#robot, #thing)+ location(#object, #place)+ in_assembly(#beam)+ supported(#beam)+ fastened(#beam, #beam, #pin).
#step = 0..numSteps.
#fluent = #inertial_fluent.

predicates
next_to(#place, #place).
fits_into(#beam, #beam).
fits_through(#beam, #beam).
is_capped_by(#beam, #beam, #beam).
holds(#fluent, #boolean, #step).
occurs(#action, #step).
success().
goal(#step).
something_happened(#step).

rules
next_to(P1,P2):- next_to(P2,P1).
-next_to(P1,P2):- not next_to(P1,P2).
holds(location(T,P1), false, I) :- holds(location(T,P2), true, I), P1!=P2.
holds(location(T1,P1), true, I) :- holds(location(R,P1), true, I), holds(in_hand(R,T2), true, I), T1=T2.
is_capped_by(B1,B2,B3):- fits_into(B1,B2), fits_into(B1,B3), B2!=B3.
is_capped_by(B1,B2,B3):- is_capped_by(B1,B3,B2).
-is_capped_by(B1,B2,B3):- not is_capped_by(B1,B2,B3).
holds(supported(B1), true, I) :- holds(in_assembly(B2), true, I), fits_into(B1,B2).
holds(supported(B1), true, I) :- holds(in_assembly(B2), true, I), fits_into(B2,B1).
-fits_into(B1,B2):- B1=B2.
-fits_through(B1,B2):- B1=B2.
-fits_through(B1,B2):- fits_into(B1,B2).
-fits_into(B1,B2):- fits_through(B1,B2).
-fits_into(B1,B2):- fits_into(B2,B1).
-fits_through(B1,B2):- fits_through(B2,B1).

holds(in_hand(R,T), false, I+1) :- occurs(putdown(R,T), I).
-occurs(putdown(R,T), I) :- not holds(in_hand(R,T), true, I).

holds(location(R,P), true, I+1) :- occurs(move(R,P), I).
-occurs(move(R,P1), I) :- holds(location(R,P2), true, I), P1=P2.
-occurs(move(R,P1), I) :- holds(location(R,P2), true, I), not next_to(P1,P2).
-occurs(move(R,P1), I) :- holds(in_hand(R,B1), true, I), holds(in_assembly(B1), true, I).
-occurs(move(R,P1), I) :- holds(in_hand(R,P1), true, I), holds(fastened(B1,B2,P), true, I).

holds(in_hand(R,T), true, I+1) :- occurs(pick_up(R,T), I).
-occurs(pick_up(R,T1), I) :- holds(location(T1,P1), true, I), holds(location(R,P2), true, I), P1!=P2.
-occurs(pick_up(R,T1), I) :- holds(in_hand(rob0,T2), true, I), #thing(T2).

holds(in_assembly(B), true, I+1) :- occurs(assemble(R,B), I).
-occurs(assemble(R,B), I) :- not holds(in_hand(R,B), true, I).
-occurs(assemble(R,B1), I) :- holds(in_assembly(B2), true, I), holds(in_assembly(B3), true, I), is_capped_by(B1,B2,B3), B2!=B3.
-occurs(assemble(R,B1), I) :- holds(in_assembly(B2), true, I), fits_through(B2,B1).
-occurs(assemble(R,B1), I) :- not holds(in_assembly(B2), true, I), holds(in_assembly(B3), true, I), is_capped_by(B2,B1,B3), B2!=B3.
-occurs(assemble(R,B), I) :- holds(location(R,P), true, I), P!=assembly_area.
-occurs(assemble(R,B), I) :- holds(in_assembly(B), true, I).
-occurs(assemble(R,B), I) :- not holds(supported(B), true, I).

holds(fastened(B1,B2,P1), true, I+1) :- occurs(fasten(R,B1,B2,P1), I).
-occurs(fasten(R,B1,B2,P1), I) :- not holds(in_assembly(B1), true, I).
-occurs(fasten(R,B1,B2,P1), I) :- not holds(in_assembly(B2), true, I).
-occurs(fasten(R,B1,B2,P1), I) :- not holds(in_hand(R,P1), true, I).
-occurs(fasten(R,B1,B2,P1), I) :- fits_into(B1,B2).
-occurs(fasten(R,B1,B2,P1), I) :- holds(fastened(B1,B2,P2), true, I).

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
goal(I) :- holds(in_assembly(b4), true, I).

% domain setup
holds(in_assembly(b1),true,0).
holds(location(b1,assembly_area),true,0).
holds(location(b2,input_area),true,0).
holds(location(b3,input_area),true,0).
holds(location(b4,input_area),true,0).
holds(location(rob0,input_area),true,0).
next_to(input_area,intermediate_area).
next_to(assembly_area,intermediate_area).
fits_into(b2, b1).
fits_into(b3, b1).
fits_into(b3, b4).
fits_into(b2, b4).
holds(in_hand(rob0,b1),false,0).
holds(in_hand(rob0,b2),false,0).
holds(in_hand(rob0,b3),false,0).
holds(in_hand(rob0,b4),false,0).
holds(in_hand(rob0,p1),false,0).
holds(in_hand(rob0,p2),false,0).
holds(in_hand(rob0,p3),false,0).
holds(in_hand(rob0,p4),false,0).

display
occurs.
