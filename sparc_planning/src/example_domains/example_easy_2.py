"""
Simple beam assmebly domain with 4 beams and 4 pins,
Beams and pins in input area,
Robot starts above_input_area
"""

from typing import Tuple,List

def generate_domain_setup() -> Tuple[List[str], List[str], List[str],List[str]]:
    """generates coarse and fine domain setups.
    Returns:
        Tuple[
            List[str], : coarse fluents
            List[str], : coarse statics
            List[str], : fine fluents
            List[str], : fine statics
            ]
    """
    coarse_fluents = [
        r'% robot location coarse',
        'holds(loc_c(rob0,intermediate_area),true,0).',
        r'% assembly relations',
        'holds(in_assembly_c(b7),true,0).',
        r'% beam and pin locations coarse',
        'holds(loc_c(p1,input_area),true,0).',
        'holds(loc_c(p2,input_area),true,0).',
        'holds(loc_c(p3,input_area),true,0).',
        'holds(loc_c(b7,assembly_area),true,0).',
        'holds(loc_c(b4,input_area),true,0).',
        'holds(loc_c(b9,input_area),true,0).',
        'holds(loc_c(b8,input_area),true,0).',
        r'% assert robots hand is empty at timestep 0',
        'holds(in_hand_c(rob0,b7),false,0).',
        'holds(in_hand_c(rob0,b4),false,0).',
        'holds(in_hand_c(rob0,b9),false,0).',
        'holds(in_hand_c(rob0,b8),false,0).',
        'holds(in_hand_c(rob0,p1),false,0).',
        'holds(in_hand_c(rob0,p2),false,0).',
        'holds(in_hand_c(rob0,p3),false,0).',
    ]
    coarse_statics = [
        r'% coarse next_to location mapping',
        'next_to_c(input_area,intermediate_area).',
        'next_to_c(assembly_area,intermediate_area).',
        'base(b7).',
            ]
    fine_fluents = coarse_fluents + [
        r'% robot location fine',
        'holds(loc_f(rob0,above_intermediate_area),true,0).',
        r'% beam and pin locations fine',
        'holds(loc_f(b7,b7t),true,0).',
        'holds(loc_f(b4,b4i),true,0).',
        'holds(loc_f(b9,b9i),true,0).',
        'holds(loc_f(b8,b8i),true,0).',
        'holds(loc_f(p1,p1i),true,0).',
        'holds(loc_f(p2,p2i),true,0).',
        'holds(in_hand_f(rob0,b4j1),false,0).',
        'holds(in_hand_f(rob0,b4j2),false,0).',
        'holds(in_hand_f(rob0,b4j3),false,0).',
        'holds(in_hand_f(rob0,b4l1),false,0).',
        'holds(in_hand_f(rob0,b4l2),false,0).',
        'holds(in_hand_f(rob0,b8j1),false,0).',
        'holds(in_hand_f(rob0,b8j2),false,0).',
        'holds(in_hand_f(rob0,b8j3),false,0).',
        'holds(in_hand_f(rob0,b8j4),false,0).',
        'holds(in_hand_f(rob0,b8j5),false,0).',
        'holds(in_hand_f(rob0,b8l1),false,0).',
        'holds(in_hand_f(rob0,b8l2),false,0).',
        'holds(in_hand_f(rob0,b8l3),false,0).',
        'holds(in_hand_f(rob0,b8l4),false,0).',
        'holds(in_hand_f(rob0,b7j1),false,0).',
        'holds(in_hand_f(rob0,b7j2),false,0).',
        'holds(in_hand_f(rob0,b7j3),false,0).',
        'holds(in_hand_f(rob0,b7j4),false,0).',
        'holds(in_hand_f(rob0,b7j5),false,0).',
        'holds(in_hand_f(rob0,b7l1),false,0).',
        'holds(in_hand_f(rob0,b7l2),false,0).',
        'holds(in_hand_f(rob0,b7l3),false,0).',
        'holds(in_hand_f(rob0,b7l4),false,0).',
        'holds(in_hand_f(rob0,b9j1),false,0).',
        'holds(in_hand_f(rob0,b9j2),false,0).',
        'holds(in_hand_f(rob0,b9j3),false,0).',
        'holds(in_hand_f(rob0,b9j4),false,0).',
        'holds(in_hand_f(rob0,b9j5),false,0).',
        'holds(in_hand_f(rob0,b9l1),false,0).',
        'holds(in_hand_f(rob0,b9l2),false,0).',
        'holds(in_hand_f(rob0,b9l3),false,0).',
        'holds(in_hand_f(rob0,b9l4),false,0).',
        'holds(in_hand_f(rob0,p1),false,0).',
        'holds(in_hand_f(rob0,p2),false,0).',
        'holds(in_hand_f(rob0,p3),false,0).',
        'holds(loc_f(p3,p3i),true,0).',
    ]
    fine_statics = coarse_statics + [
        r'% next_to_f location map',
        'next_to_f(above_input_area,above_intermediate_area).',
        'next_to_f(above_assembly_area,above_intermediate_area).',
        'component(input_area,above_input_area).',
        'component(intermediate_area,above_intermediate_area).',
        'component(assembly_area,above_assembly_area).',
        ]
    return coarse_fluents, coarse_statics, fine_fluents, fine_statics
