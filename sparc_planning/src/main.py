import copy
from beam_domain_coarse import generate_coarse_beam_domain
from beam_domain_fine import generate_fine_beam_domain
from example_domains.example_1 import generate_domain_setup
import logging
import asyncio
from al_structures import ActionInstance, GoalDefinition
from planning import plan
from sparc_io import extract_states_from_answer_set
from zooming import remove_chars_from_last_number, zoom
import re

# set up logging
logger = logging.basicConfig(level=logging.INFO, 
                             format='%(asctime)s - %(filename)s:%(lineno)s - %(levelname)s - %(message)s',
                             datefmt='%m-%d %H:%M',)

async def main():
    coarse = generate_coarse_beam_domain()
    #coarse.save_AL('/home/local/MTC_ORI_Collab/sparc_planning/action_lang_files/coarse_beam_AL.txt')

    fine = generate_fine_beam_domain()
    #fine.save_AL('/home/local/MTC_ORI_Collab/sparc_planning/action_lang_files/fine_beam_AL.txt')

    coarse_fluents, coarse_statics, fine_fluents, fine_statics = generate_domain_setup()
    coarse.domain_setup = coarse_fluents + coarse_statics
    fine.domain_setup = fine_fluents + fine_statics

    # set a goal and create coarse sparc prog.
    coarse.goal_description = [GoalDefinition('in_assembly_c',['b4'],True)]
    coarse_prog = coarse.to_sparc_program()
    coarse_prog.save('/home/local/MTC_ORI_Collab/sparc_planning/sparc_files/temp.sp')

    #run coarse planner
    coarse_plan = await plan('/home/local/MTC_ORI_Collab/sparc_planning/sparc_files/temp.sp', max_length=10, min_length=1)

    #collect results
    coarse_states, coarse_actions = extract_states_from_answer_set(coarse_plan[0]) #[0] just takes first answer set (valid plan), further work could explore which route to take
    logging.info(coarse_actions)

    all_fine_actions = []
    fine_plan_length = 0

    fine_state_fluents = copy.deepcopy(fine_fluents)
    for i in range(len(coarse_actions)):
        _, course_action_name, *objs = re.split('\(|\)|\,', coarse_actions[i])[:-3]
        
        #ZOOM
        logging.info(f'Coarse Action: {course_action_name}')
        logging.info(f'Action objects: {objs}')
        actr = ActionInstance(coarse.get_action_by_name(course_action_name), objs, coarse_states[i].time_step)
        logging.info('Building zoomed system description...')
        zoomed_fine_res_description = zoom(coarse_states[i], coarse_states[i+1], actr, coarse, fine, True)
        fine_prog = zoomed_fine_res_description.to_sparc_program()
        fine_prog.save('/home/local/MTC_ORI_Collab/sparc_planning/sparc_files/fine_temp.sp')
        logging.info('Zooming complete')

        #run fine planner
        fine_plan = await plan('/home/local/MTC_ORI_Collab/sparc_planning/sparc_files/fine_temp.sp',
                                max_length=fine_plan_length+10,
                                min_length=fine_plan_length)
        #collect results
        fine_states, fine_actions = extract_states_from_answer_set(fine_plan[0])
        logging.info(fine_actions)
        fine_plan_length += len(fine_actions)
        # update fine state history
        # only recieve a partial state back from zoomed fine res system
        # need to update full state, so un-used fluents need to be progressed in time (did not change)
        updated_fluents = fine_states[-1].fluents
        # state strings are timestamped so this needs to be fixed for comparison
        s1flu = [remove_chars_from_last_number(f1) for f1 in fine_state_fluents if f1[0] != '%'] # if f1[0] != '%' ignores comment lines
        s2flu = [remove_chars_from_last_number(f2) for f2 in updated_fluents]
        s1funcs, s1fun_vals = [], []
        for func in s1flu:
            split = re.split('\(|\)|\,',func)
            s1funcs.append(split[0]+'('+split[1]+'('+','.join(split[2:-3])+')')
            s1fun_vals.append(split[-2])
        s2funcs = []
        for func in s2flu:
            split = re.split('\(|\)|\,',func)
            s2funcs.append(split[0]+'('+split[1]+'('+','.join(split[2:-3])+')')
        
        # find fluents which havent been updated in zoomed description
        # functions in s1 but not in s2
        funcs = {*s1funcs}.difference({*s2funcs})
        unchanged_fluents = [f'{f},{s1fun_vals[s1funcs.index(f)]},{fine_plan_length}).' for f in funcs]
        # update fluents at this step
        fine_state_fluents = unchanged_fluents + updated_fluents
        # update fine domain for next planning step
        fine.domain_setup = fine_statics + fine_state_fluents 
        fine.start_step = fine_plan_length
        all_fine_actions += fine_actions

    logging.info(f'Coarse Actions: {coarse_actions}')
    logging.info(f'Fine Actions: {all_fine_actions}')

if __name__ == "__main__":
    asyncio.run(main())