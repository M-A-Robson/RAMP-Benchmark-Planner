from beam_domain_coarse import generate_coarse_beam_domain
from beam_domain_fine import generate_fine_beam_domain
import logging
import asyncio
from al_structures import ActionInstance, GoalDefinition
from planning import plan
from sparc_io import extract_states_from_answer_set
from zooming import zoom
import re

# set up logging
logger = logging.basicConfig(level=logging.DEBUG, 
                             format='%(asctime)s - %(filename)s:%(lineno)s - %(levelname)s - %(message)s',
                             datefmt='%m-%d %H:%M',)

async def main():
    coarse = generate_coarse_beam_domain()
    #coarse.save_AL('/home/local/MTC_ORI_Collab/sparc_planning/action_lang_files/coarse_beam_AL.txt')

    fine = generate_fine_beam_domain()
    #fine.save_AL('/home/local/MTC_ORI_Collab/sparc_planning/action_lang_files/fine_beam_AL.txt')

    coarse_domain_setup = [
        r'% robot location coarse',
        'holds(loc_c(rob0,input_area),true,0).',
        r'% assembly relations',
        'holds(in_assembly_c(b1),true,0).',
        'holds(in_assembly_c(b2),true,0).',
        'holds(in_assembly_c(b3),true,0).',
        r'% beam and pin locations coarse',
        'holds(loc_c(p1,assembly_area),true,0).',
        'holds(loc_c(p2,assembly_area),true,0).',
        'holds(loc_c(p3,input_area),true,0).',
        'holds(loc_c(p4,input_area),true,0).',
        'holds(loc_c(b1,assembly_area),true,0).',
        'holds(loc_c(b2,assembly_area),true,0).',
        'holds(loc_c(b3,assembly_area),true,0).',
        'holds(loc_c(b4,input_area),true,0).',
        r'% coarse fastening status',
        'holds(fastened_c(b1,b2,p1),true,0).',
        'holds(fastened_c(b1,b3,p2),true,0).',
        r'% coarse next_to location mapping',
        'next_to_c(input_area,intermediate_area).',
        'next_to_c(assembly_area,intermediate_area).',
        r'% coarse beam relations',
        'fits_into_c(b2,b1).',
        'fits_into_c(b3,b1).',
        'fits_into_c(b3,b4).',
        'fits_into_c(b2,b4).',
        r'% assert robots hand is empty at timestep 0',
        'holds(in_hand_c(rob0,b1),false,0).',
        'holds(in_hand_c(rob0,b2),false,0).',
        'holds(in_hand_c(rob0,b3),false,0).',
        'holds(in_hand_c(rob0,b4),false,0).',
        'holds(in_hand_c(rob0,p1),false,0).',
        'holds(in_hand_c(rob0,p2),false,0).',
        'holds(in_hand_c(rob0,p3),false,0).',
        'holds(in_hand_c(rob0,p4),false,0).',
    ]
    coarse.domain_setup = coarse_domain_setup

    fine_domain_setup = coarse_domain_setup + [
        r'% robot location fine',
        'holds(loc_f(rob0,above_input_area),true,0).',
        r'% beam and pin locations fine',
        'holds(loc_f(b1,b1t),true,0).',
        'holds(loc_f(b2,b2t),true,0).',
        'holds(loc_f(b3,b3i),true,0).',
        'holds(loc_f(b4,b4i),true,0).',
        'holds(loc_f(p1,p1i),true,0).',
        'holds(loc_f(p2,p2i),true,0).',
        'holds(loc_f(p3,p3i),true,0).',
        'holds(loc_f(p4,p4i),true,0).',
        r'% fine fastening status',
        'holds(fastened_f(j1,j3,p1),true,0)',
        'holds(fastened_f(j2,j5,p2),true,0)',
        r'% beam components and connections',
        'connected_to(j1,l1).',
        'connected_to(l1,j2).',
        'component(b1,j1).',
        'component(b1,l1).',
        'component(b1,j2).',
        'connected_to(j3,l2).',
        'connected_to(l2,j4).',
        'component(b2,j3).',
        'component(b2,l2).',
        'component(b2,j4).',
        'connected_to(j5,l3).',
        'connected_to(l3,j6).',
        'component(b3,j5).',
        'component(b3,l3).',
        'component(b3,j6).',
        'connected_to(j7,l4).',
        'connected_to(l4,j8).',
        'component(b4,j7).',
        'component(b4,l4).',
        'component(b4,j8).',
        r'% next_to_f location map',
        'next_to_f(above_input_area,b2i).',
        'next_to_f(above_input_area,b3i).',
        'next_to_f(above_input_area,b4i).',
        'next_to_f(above_input_area,p1i).',
        'next_to_f(above_input_area,p2i).',
        'next_to_f(above_input_area,p3i).',
        'next_to_f(above_input_area,p4i).',
        'next_to_f(above_input_area,above_intermediate_area).',
        'next_to_f(above_assembly_area,above_intermediate_area).',
        'next_to_f(above_assembly_area,b2a).',
        'next_to_f(above_assembly_area,b3a).',
        'next_to_f(above_assembly_area,b4a).',
        'next_to_f(above_assembly_area,b2t).',
        'next_to_f(above_assembly_area,b3t).',
        'next_to_f(above_assembly_area,b4t).',
        'next_to_f(above_assembly_area,p1a).',
        'next_to_f(above_assembly_area,p2a).',
        'next_to_f(above_assembly_area,p3a).',
        'next_to_f(above_assembly_area,p4a).',
        'next_to_f(above_assembly_area,nt_b2t).',
        'next_to_f(above_assembly_area,nt_b3t).',
        'next_to_f(above_assembly_area,nt_b4t).',
        'next_to_f(b2t,b2a).',
        'next_to_f(b3t,b3a).',
        'next_to_f(b4t,b4a).',
        'next_to_f(p1t,p1a).',
        'next_to_f(p2t,p2a).',
        'next_to_f(p3t,p3a).',
        'next_to_f(p4t,p4a).',
        r'% location component relations',
        'component(input_area,b2i).',
        'component(input_area,b3i).',
        'component(input_area,b4i).',
        'component(input_area,p1i).',
        'component(input_area,p2i).',
        'component(input_area,p3i).',
        'component(input_area,p4i).',
        'component(input_area,above_input_area).',
        'component(intermediate_area,above_intermediate_area).',
        'component(assembly_area,above_assembly_area).',
        'component(assembly_area,b2t).',
        'component(assembly_area,b2a).',
        'component(assembly_area,b3t).',
        'component(assembly_area,b3a).',
        'component(assembly_area,b4t).',
        'component(assembly_area,b4a).',
        'component(assembly_area,p1t).',
        'component(assembly_area,p1a).',
        'component(assembly_area,p2t).',
        'component(assembly_area,p2a).',
        'component(assembly_area,p3t).',
        'component(assembly_area,p3a).',
        'component(assembly_area,p4t).',
        'component(assembly_area,p4a).',
        r'% fine beam to beam connections',
        'fits_into_f(j3,j1).',
        'fits_into_f(j5,j2).',
        'fits_into_f(j6,j8).',
        'fits_into_f(j4,j7).',
        r'% beam assembly location mapping',
        'assem_approach_loc(b2,b2a).',
        'assem_approach_loc(b3,b3a).',
        'assem_approach_loc(b4,b4a).',
        'assem_target_loc(b2,b2t).',
        'assem_target_loc(b3,b3t).',
        'assem_target_loc(b4,b4t).',
        r'% pin assembly location mapping',
        'assem_approach_loc(p1,p1a).',
        'assem_approach_loc(p2,p2a).',
        'assem_approach_loc(p3,p3a).',
        'assem_approach_loc(p4,p4a).',
        'assem_target_loc(p1,p1t).',
        'assem_target_loc(p2,p2t).',
        'assem_target_loc(p3,p3t).',
        'assem_target_loc(p4,p4t).',
        r'% near to mapping',
        'near_to(nt_b2t,b2t).',
        'near_to(nt_b3t,b3t).',
        'near_to(nt_b4t,b4t).',
        r'% assert hand is empty at timestep 0',
        'holds(in_hand_f(rob0,l1),false,0).',
        'holds(in_hand_f(rob0,l2),false,0).',
        'holds(in_hand_f(rob0,l3),false,0).',
        'holds(in_hand_f(rob0,l4),false,0).',
        'holds(in_hand_f(rob0,j3),false,0).',
        'holds(in_hand_f(rob0,j4),false,0).',
        'holds(in_hand_f(rob0,j7),false,0).',
        'holds(in_hand_f(rob0,j8),false,0).',
        'holds(in_hand_f(rob0,j1),false,0).',
        'holds(in_hand_f(rob0,j2),false,0).',
        'holds(in_hand_f(rob0,j5),false,0).',
        'holds(in_hand_f(rob0,j6),false,0).',
        'holds(in_hand_f(rob0,p1),false,0).',
        'holds(in_hand_f(rob0,p2),false,0).',
        'holds(in_hand_f(rob0,p3),false,0).',
        'holds(in_hand_f(rob0,p4),false,0).',
    ]
    fine.domain_setup = fine_domain_setup

    # set a goal and create coarse sparc prog.
    coarse.goal_description = [GoalDefinition('in_assembly_c',['b4'],True)]
    coarse_prog = coarse.to_sparc_program()
    coarse_prog.save('/home/local/MTC_ORI_Collab/sparc_planning/sparc_files/temp.sp')

    #run coarse planner
    res = await plan('/home/local/MTC_ORI_Collab/sparc_planning/sparc_files/temp.sp', max_length=10, min_length=1)

    #collect results
    states, actions = extract_states_from_answer_set(res[0])
    logging.info(actions)
    _, action_name, *objs = re.split('\(|\)|\,', actions[0])[:-3]

    logging.info(f'Action: {action_name}')
    logging.info(f'Action objects: {objs}')
    actr = ActionInstance(coarse.get_action_by_name(action_name), objs, states[0].time_step)

    zoomed_fine_res_description = zoom(states[0], states[1], actr, coarse, fine, True)
    fine_prog = zoomed_fine_res_description.to_sparc_program()
    fine_prog.save('/home/local/MTC_ORI_Collab/sparc_planning/sparc_files/fine_temp.sp')


if __name__ == "__main__":
    asyncio.run(main())