%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SPARC file for beam domain
%% Author: MARK ROBSON
%% Description: BEAM DOMAIN
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

%statics
fits_into(#beam, #beam).
fits_through(#beam, #beam).
is_capped_by(#beam, #beam, #beam).

rules
% casual rules
val(in_assembly(B), true, I+1) :- occurs(add(B), I).

% state constraints
% define support - a beam can be supported if either of its ends can phsyically
% connected to a beam already in the assembly.
val(supported(B1), true, I) :- val(in_assembly(B2), true, I), fits_into(B1, B2).
val(supported(B1), true, I) :- val(in_assembly(B2), true, I), fits_into(B2, B1).

% define capped
is_capped_by(B1, B2, B3) :- fits_into(B1,B2), fits_into(B1,B3), B2!=B3.
is_capped_by(B1, B2, B3) :- is_capped_by(B1, B3, B2).

% these rules govern legal interactions between beams
% cant in-ception
-fits_into(B1,B2) :- fits_into(B2,B1).
-fits_through(B1,B2) :- fits_through(B2,B1).
% through and in are mutually exclusive
-fits_through(B1, B2) :- fits_into(B1, B2).
-fits_into(B1, B2) :- fits_through(B1,B2).
% beams cannot self intersect
-fits_into(B1,B2) :- B1=B2.
-fits_through(B1,B2) :- B1=B2.

% executability conditions
% cannot add beams already in assembly
-occurs(add(B1), I) :- val(in_assembly(B1), true, I).
% cannot add beam if both cap beams are already in place
-occurs(add(B1), I) :- val(in_assembly(B3), true, I), val(in_assembly(B2), true, I), is_capped_by(B1, B2, B3), B3!=B2.
% cannot add beam if another beam which needs to pass through the beam is alread in the assembly
-occurs(add(B1), I) :- val(in_assembly(B2), true, I), fits_through(B2, B1).
% cannot (should not) add a beam if it needs to have assembled beams place into it, unless it caps those beams
-occurs(add(B1), I) :- val(in_assembly(B2), true, I), fits_into(B2,B1), -is_capped_by(B2,B1,B3).
% cannot add a beam which would cap another beam if the beams to be capped are not yet in the assembly
-occurs(add(B1), I) :- not val(in_assembly(B2), true, I), val(in_assembly(B3), true, I), is_capped_by(B2, B1, B3), B2!=B3.
% cannot add a beam unless it fits into to another beam already in the assembly 
% this constraint could be relaxed with suitable fixturing
-occurs(add(B1), I) :- not val(supported(B1), true, I).

%% Fluents can only have one value at a time...
-val(F, V2, I) :- val(F, V1, I), V1!=V2.

%inertia
val(F, Y, I+1) :- #inertial_fluent(F),
             	  val(F, Y, I),
                  not -val(F, Y, I+1), I < numSteps.

%% CWA for Actions
-occurs(A,I) :- not occurs(A,I).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Planning and goal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Failure is not an option...
success :- goal(I), I <= numSteps.
:- not success. 

%% Cannot stop executing actions, until goal achieved...
occurs(A, I) | -occurs(A, I) :- not goal(I).

%% Cannot have two actions happening concurrently
-occurs(A2, I) :- occurs(A1, I), A1 != A2. 

something_happened(I) :- occurs(A, I).

:- not goal(I), not something_happened(I).
   
% goal description   
goal(I) :- val(in_assembly(d6), true, I).

% domain set up
% always start with base beam in place
val(in_assembly(d1), true, 0).

% simple square (1)
fits_into(d2, d1).
fits_into(d3, d1).
fits_into(d3, d4).
fits_into(d2, d4).

display
occurs.
