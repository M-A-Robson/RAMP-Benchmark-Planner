%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% AutoGenerated SPARC file
%% Author: MARK ROBSON 2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#const numSteps = 12.

sorts
#beam = [d][1..13].
#object = #beam.
#boolean = {true, false}.
#inertial_fluent = in_assembly(#beam) + supported(#beam).
#fluent = #inertial_fluent.
#step = 0..numSteps.
#action = add(#beam).

predicates
val(#fluent, #boolean, #step).
occurs(#action, #step).
success().
goal(#step).
something_happened(#step).
fits_into(#beam, #beam).
fits_through(#beam, #beam).
is_capped_by(#beam, #beam, #beam).

rules
val(in_assembly(B), true, I+1) :- occurs(add(B), I).
val(supported(B1), true, I) :- val(in_assembly(B2), true, I), fits_into(B1, B2).
val(supported(B1), true, I) :- val(in_assembly(B2), true, I), fits_into(B2, B1).
is_capped_by(B1, B2, B3) :- fits_into(B1,B2), fits_into(B1,B3), B2!=B3.
is_capped_by(B1, B2, B3) :- is_capped_by(B1, B3, B2).
-fits_into(B1,B2) :- fits_into(B2,B1).
-fits_through(B1,B2) :- fits_through(B2,B1).
-fits_through(B1, B2) :- fits_into(B1, B2).
-fits_into(B1, B2) :- fits_through(B1,B2).
-fits_into(B1,B2) :- B1=B2.
-fits_through(B1,B2) :- B1=B2.
-occurs(add(B1), I) :- val(in_assembly(B1), true, I).
-occurs(add(B1), I) :- val(in_assembly(B3), true, I), val(in_assembly(B2), true, I), is_capped_by(B1, B2, B3), B3!=B2.
-occurs(add(B1), I) :- val(in_assembly(B2), true, I), fits_through(B2, B1).
-occurs(add(B1), I) :- val(in_assembly(B2), true, I), fits_into(B2,B1), -is_capped_by(B2,B1,B3).
-occurs(add(B1), I) :- not val(in_assembly(B2), true, I), val(in_assembly(B3), true, I), is_capped_by(B2, B1, B3), B2!=B3.
-occurs(add(B1), I) :- not val(supported(B1), true, I).
-val(F, V2, I) :- val(F, V1, I), V1!=V2.
val(F, Y, I+1) :- #inertial_fluent(F),
             	  val(F, Y, I),
                  not -val(F, Y, I+1), I < numSteps.
-occurs(A,I) :- not occurs(A,I).
success :- goal(I), I <= numSteps.
:- not success.
occurs(A, I) | -occurs(A, I) :- not goal(I).
-occurs(A2, I) :- occurs(A1, I), A1 != A2.
something_happened(I) :- occurs(A, I).
:- not goal(I), not something_happened(I).
goal(I) :- val(in_assembly(d4), true, I).
val(in_assembly(d1), true, 0).
fits_into(d2, d1).
fits_into(d3, d1).
fits_into(d3, d4).
fits_into(d2, d4).

display
occurs.
