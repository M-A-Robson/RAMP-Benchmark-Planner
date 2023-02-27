%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% AutoGenerated Beam Description file
%% Author: MARK ROBSON 2023
%% for example purposes only
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% coarse data
#beam = {b1,b2,b3,b4}.
#pin = {p1,p2,p3,p4}.
fits_into_c(b2,b1).
fits_into_c(b3,b1).
fits_into_c(b2,b4).
fits_into_c(b3,b4).

% fine data
#beam = {b1,b2,b3,b4}.
#pin = {p1,p2,p3,p4}.
#link = {L1,L2,L3,L4}.
#in_m_end = {J3,J4,J5,J6}.
#in_f_end = {J1,J2,J7,J8}.
#input_location = {b1i,b2i,b3i,b4i,p1i,p2i,p3i,p4i}.
#target_location = {b1t,b2t,b3t,b4t,p1t,p2t,p3t,p4t}.
#approach_location = {b1a,b2a,b3a,b4a,p1a,p2a,p3a,p4a}.
#through_location = {dummy_through_location}.
#prerot_location = {dummy_prerot_location}.
#near_to_location = {nt_b1i,nt_b2i,nt_b3i,nt_b4i,nt_p1i,nt_p2i,nt_p3i,nt_p4i,nt_b1t,nt_b2t,nt_b3t,nt_b4t,nt_p1t,nt_p2t,nt_p3t,nt_p4t,nt_b1a,nt_b2a,nt_b3a,nt_b4a,nt_p1a,nt_p2a,nt_p3a,nt_p4a}.
fits_into_f(J3,J1).
fits_into_f(J5,J2).
fits_into_f(J4,J7).
fits_into_f(J6,J8).
connected_to(J1,L1).
connected_to(L1,J2).
component(b1,J1).
component(b1,L1).
component(b1,J2).
connected_to(J3,L2).
connected_to(L2,J4).
component(b2,J3).
component(b2,L2).
component(b2,J4).
connected_to(J5,L3).
connected_to(L3,J6).
component(b3,J5).
component(b3,L3).
component(b3,J6).
connected_to(J7,L4).
connected_to(L4,J8).
component(b4,J7).
component(b4,L4).
component(b4,J8).
near_to(nt_b1i,b1i).
near_to(nt_b2i,b2i).
near_to(nt_b3i,b3i).
near_to(nt_b4i,b4i).
near_to(nt_p1i,p1i).
near_to(nt_p2i,p2i).
near_to(nt_p3i,p3i).
near_to(nt_p4i,p4i).
near_to(nt_b1t,b1t).
near_to(nt_b2t,b2t).
near_to(nt_b3t,b3t).
near_to(nt_b4t,b4t).
near_to(nt_p1t,p1t).
near_to(nt_p2t,p2t).
near_to(nt_p3t,p3t).
near_to(nt_p4t,p4t).
near_to(nt_b1a,b1a).
near_to(nt_b2a,b2a).
near_to(nt_b3a,b3a).
near_to(nt_b4a,b4a).
near_to(nt_p1a,p1a).
near_to(nt_p2a,p2a).
near_to(nt_p3a,p3a).
near_to(nt_p4a,p4a).
component(input_area,b1i).
component(input_area,b2i).
component(input_area,b3i).
component(input_area,b4i).
component(input_area,p1i).
component(input_area,p2i).
component(input_area,p3i).
component(input_area,p4i).
component(assembly_area,b1t).
component(assembly_area,b2t).
component(assembly_area,b3t).
component(assembly_area,b4t).
component(assembly_area,p1t).
component(assembly_area,p2t).
component(assembly_area,p3t).
component(assembly_area,p4t).
component(assembly_area,b1a).
component(assembly_area,b2a).
component(assembly_area,b3a).
component(assembly_area,b4a).
component(assembly_area,p1a).
component(assembly_area,p2a).
component(assembly_area,p3a).
component(assembly_area,p4a).
next_to_f(b1t,b1a).
next_to_f(b2t,b2a).
next_to_f(b3t,b3a).
next_to_f(b4t,b4a).
next_to_f(p1t,p1a).
next_to_f(p2t,p2a).
next_to_f(p3t,p3a).
next_to_f(p4t,p4a).
next_to_f(above_assembly_area,b1a).
next_to_f(above_assembly_area,b2a).
next_to_f(above_assembly_area,b3a).
next_to_f(above_assembly_area,b4a).
next_to_f(above_assembly_area,p1a).
next_to_f(above_assembly_area,p2a).
next_to_f(above_assembly_area,p3a).
next_to_f(above_assembly_area,p4a).
next_to_f(above_assembly_area,b1t).
next_to_f(above_assembly_area,b2t).
next_to_f(above_assembly_area,b3t).
next_to_f(above_assembly_area,b4t).
next_to_f(above_assembly_area,p1t).
next_to_f(above_assembly_area,p2t).
next_to_f(above_assembly_area,p3t).
next_to_f(above_assembly_area,p4t).
next_to_f(above_assembly_area,nt_b1t).
next_to_f(above_assembly_area,nt_b2t).
next_to_f(above_assembly_area,nt_b3t).
next_to_f(above_assembly_area,nt_b4t).
next_to_f(above_assembly_area,nt_p1t).
next_to_f(above_assembly_area,nt_p2t).
next_to_f(above_assembly_area,nt_p3t).
next_to_f(above_assembly_area,nt_p4t).
next_to_f(above_assembly_area,nt_b1a).
next_to_f(above_assembly_area,nt_b2a).
next_to_f(above_assembly_area,nt_b3a).
next_to_f(above_assembly_area,nt_b4a).
next_to_f(above_assembly_area,nt_p1a).
next_to_f(above_assembly_area,nt_p2a).
next_to_f(above_assembly_area,nt_p3a).
next_to_f(above_assembly_area,nt_p4a).
next_to_f(above_input_area,nt_b1i).
next_to_f(above_input_area,nt_b2i).
next_to_f(above_input_area,nt_b3i).
next_to_f(above_input_area,nt_b4i).
next_to_f(above_input_area,nt_p1i).
next_to_f(above_input_area,nt_p2i).
next_to_f(above_input_area,nt_p3i).
next_to_f(above_input_area,nt_p4i).
next_to_f(above_input_area,b1i).
next_to_f(above_input_area,b2i).
next_to_f(above_input_area,b3i).
next_to_f(above_input_area,b4i).
next_to_f(above_input_area,p1i).
next_to_f(above_input_area,p2i).
next_to_f(above_input_area,p3i).
next_to_f(above_input_area,p4i).
