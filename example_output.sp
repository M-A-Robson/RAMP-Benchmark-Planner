connected_to(J1,L1).
connected_to(L1,J2).
component(J1,beam1).
component(L1,beam1).
component(J2,beam1).
connected_to(J3,L2).
connected_to(L2,J4).
component(J3,beam2).
component(L2,beam2).
component(J4,beam2).
connected_to(J5,L3).
connected_to(L3,J6).
component(J5,beam3).
component(L3,beam3).
component(J6,beam3).
connected_to(J7,L4).
connected_to(L4,J8).
component(J7,beam4).
component(L4,beam4).
component(J8,beam4).
link = {L1,L2,L3,L4}.
in-m-end = {J3,J4,J7,J8}.
in-f-end = {J1,J2,J5,J6}.
beam = {beam1,beam2,beam3,beam4}.
