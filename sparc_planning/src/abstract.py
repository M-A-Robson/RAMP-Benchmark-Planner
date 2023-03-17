import os
import copy
from beam_domain_abstract import generate_abstract_beam_domain
from beam_domain_coarse import generate_coarse_beam_domain
from beam_domain_fine import generate_fine_beam_domain
# from example_domains.example_latest import generate_domain_setup
# from example_domains.example_easy_1 import generate_domain_setup
from example_domains.example_easy_2 import generate_domain_setup
# from example_domains.example_easy_3 import generate_domain_setup
import logging
import asyncio
from al_structures import ActionInstance, GoalDefinition, StateConstraint
from planning import plan
from sparc_io import extract_states_from_answer_set
from zooming import remove_chars_from_last_number, zoom
import re
import time

from beam_assembly.beam_assembly_parser import load_beam_xml, load_assembly_xml
from beam_assembly.beam_to_sparc import create_sparc_data


# MIN_COARSE_PLAN_LENGTH = 50
# MAX_COARSE_PLAN_LENGTH = 55
# MIN_COARSE_PLAN_LENGTH = 30
MAX_COARSE_PLAN_LENGTH = 40
MAX_COARSE_PLAN_LENGTH = 55
FINE_ITERATION_MIN = 1
FINE_ITERATION_LIMIT = 15
COARSE_ITERATION_MIN = 5
COARSE_PLAN_ITERATION_LIMIT = 32

# colorised logger from https://stackoverflow.com/questions/384076/how-can-i-color-python-logging-output
class CustomFormatter(logging.Formatter):
    grey = "\x1b[38;20m"
    yellow = "\x1b[33;20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    format = "%(asctime)s - %(filename)s:%(lineno)s - %(levelname)s - %(message)s"
    datefmt = "%m-%d %H:%M"

    FORMATS = {
        logging.DEBUG: grey + format + reset,
        logging.INFO: grey + format + reset,
        logging.WARNING: yellow + format + reset,
        logging.ERROR: red + format + reset,
        logging.CRITICAL: bold_red + format + reset
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)
    
# set up logging
logger = logging.getLogger()
logger.setLevel(logging.INFO)

# create console handler
ch = logging.StreamHandler()
ch.setLevel(logging.INFO)
ch.setFormatter(CustomFormatter())
logger.addHandler(ch)

async def main():

    # new high level abstract domain
    abstract = generate_abstract_beam_domain()
    
    coarse = generate_coarse_beam_domain()
    # coarse.save_AL('/home/local/MTC_ORI_Collab/sparc_planning/action_lang_files/coarse_beam_AL.txt')
    #s = coarse.to_sparc_program()
    #s.save('/home/local/MTC_ORI_Collab/sparc_planning/sparc_files/beam_domain_coarse.sp')

    fine = generate_fine_beam_domain()
    # fine.save_AL('/home/local/MTC_ORI_Collab/sparc_planning/action_lang_files/fine_beam_AL.txt')

    beams = load_beam_xml(os.path.join(os.environ['PLANNER_PATH'], "example_beamset_latest.xml"))
    # assem = load_assembly_xml(beams, os.path.join(os.environ['PLANNER_PATH'], "assembly_latest.xml"))
    # assem = load_assembly_xml(beams, os.path.join(os.environ['PLANNER_PATH'], "assembly_easy_1.xml"))
    assem = load_assembly_xml(beams, os.path.join(os.environ['PLANNER_PATH'], "assembly_easy_2.xml"))
    
    # scene = assem.create_display_scene()
    # scene.show()

    # generate sparc planner data from xml
    coarse_sorts, xml_coarse_statics, fine_sorts, xml_fine_statics = create_sparc_data(assem)
    coarse_sort_dict = dict(zip([s.name for s in coarse_sorts], coarse_sorts))
    fine_sort_dict = dict(zip([s.name for s in fine_sorts], fine_sorts))

    logging.debug(f'Fine sorts: {[s.name for s in fine_sorts]}')

    # override abstract sorts data
    for sort in abstract.sorts: # only "beam"
        if sort.name in coarse_sort_dict.keys():
            sort.instances = coarse_sort_dict[sort.name].instances
    # override coarse sorts with xml derived data
    for sort in coarse.sorts:
        if sort.name in coarse_sort_dict.keys():
            sort.instances = coarse_sort_dict[sort.name].instances
    # override fine sorts with xml derived data
    for sort in fine.sorts:
        if sort.name in fine_sort_dict.keys():
            sort.instances = fine_sort_dict[sort.name].instances

    # abstract domain setup assumes base beam is in_assembly.
    abstract.domain_setup = [f'holds(in_assembly_c({assem.base.name}),true,0).'] + xml_coarse_statics

    # get domain setup data from python script
    coarse_fluents, example_coarse_statics, fine_fluents, example_fine_statics = generate_domain_setup()
    
    # combine with xml derived data
    coarse_statics = xml_coarse_statics + example_coarse_statics
    fine_statics = xml_fine_statics + example_fine_statics

    # update domain setup
    coarse.domain_setup = coarse_fluents + coarse_statics
    fine.domain_setup = fine_fluents + fine_statics

    # automatically generate goal definition for abstract level (add all beams to assembly)
    for beam in assem.beams:
        if beam == assem.base:
            continue
        abstract.goal_description.append(
            GoalDefinition('in_assembly_c',[beam.name],True)
        )

    abs_prog = abstract.to_sparc_program()
    abs_prog.save(os.path.join(os.environ['PLANNER_PATH'], 'sparc_planning/sparc_files/abs_temp.sp'))

    t_start_abs = time.perf_counter()
    #run abstract planner (we know length because it should match number of beams to insert)
    abstract_plan = await plan(os.path.join(os.environ['PLANNER_PATH'], 'sparc_planning/sparc_files/abs_temp.sp'),
                             max_length=len(assem.beams), min_length=len(assem.beams)-1)
    
    #collect results
    abstract_states, abstract_actions = extract_states_from_answer_set(abstract_plan[0]) #[0] just takes first answer set (valid plan), further work could explore which route to take
    t_stop_abs = time.perf_counter()
    logging.info(abstract_actions)
    #logging.info(t_stop_abs-t_start_abs)

    # set a goal and create coarse sparc prog.
    # coarse.goal_description = [
    #     GoalDefinition('in_assembly_c',['b4'],True),
    #     GoalDefinition('fastened_c',['b7','b4','p1'],True),
    #     GoalDefinition('in_assembly_c',['b5'],True),
    #     GoalDefinition('fastened_c',['b7','b5','p2'],True),
    #     GoalDefinition('in_assembly_c',['b8'],True),
    #     GoalDefinition('fastened_c',['b5','b8','p3'],True),
    #     GoalDefinition('fastened_c',['b4','b8','p4'],True),
    #     ] # full coarse plan is 50 steps, a good lower bound heuristic is 6 to 7 steps per assembly goal
        

    # iteratively search for coarse plan based on abstract beam ordering
    t_start_coarse = time.perf_counter()
    coarse_plan_length = 0
    all_coarse_actions = []
    all_coarse_states = []
    coarse_goals = []
    for step in abstract_actions:
        inserted_beam = re.split('\(|\)|\,', step)[2]
        coarse_goals.append([GoalDefinition('in_assembly_c',[inserted_beam],True)])
    fastening_goal = []
    for i in range(len(assem.connections)):
        conn = assem.connections[i]
        B1 = conn.element1.name
        B2 = conn.element2.name
        fastening_goal.append(GoalDefinition('fastened_c',[B1,B2,f'P{i}'],True))
    coarse_goals.append(fastening_goal)

    for goal in coarse_goals:
        # extend goal
        coarse.goal_description += goal 
        coarse_prog = coarse.to_sparc_program()
        coarse_prog.save(os.path.join(os.environ['PLANNER_PATH'], 'sparc_planning/sparc_files/temp.sp'))
        # run coarse planner
        coarse_plan = await plan(os.path.join(os.environ['PLANNER_PATH'], 'sparc_planning/sparc_files/temp.sp'),
                                max_length=coarse_plan_length+COARSE_PLAN_ITERATION_LIMIT,
                                min_length=coarse_plan_length+COARSE_ITERATION_MIN)
        # collect results
        coarse_states, coarse_actions = extract_states_from_answer_set(coarse_plan[0], min_time_step=coarse_plan_length) #[0] just takes first answer set (valid plan), further work could explore which route to take
        logging.info(coarse_actions)
        coarse_plan_length += len(coarse_actions)
        all_coarse_actions += coarse_actions
        all_coarse_states += coarse_states[:-1] # last state will be same as first state of next loop
        # override state for next iteration start
        coarse.domain_setup = coarse_states[-1].fluents + coarse_statics
        coarse.start_step = coarse_plan_length

    all_coarse_states.append(coarse_states[-1]) # add end state data
    t_stop_coarse = time.perf_counter()
    #logging.info(t_stop_coarse-t_start_coarse)
    #logging.info(all_coarse_actions)

    # # post full fine definition to file for debugging
    # fine_prog_test = fine.to_sparc_program()
    # fine_prog_test.save(os.path.join(os.environ['PLANNER_PATH'], 'sparc_planning/sparc_files/fine_unzoomed_temp.sp'))

    all_fine_actions = []
    fine_plan_length = 0

    fine_state_fluents = copy.deepcopy(fine_fluents)
    t_start_fine = time.perf_counter()
    fine_plan_success = False
    try:
        for i in range(len(all_coarse_actions)):
            logging.debug(f'FINE LOOP {i}')
            _, course_action_name, *objs = re.split('\(|\)|\,', all_coarse_actions[i])[:-3]
            
            #ZOOM
            logging.info(f'Coarse Action: {course_action_name}')
            logging.info(f'Action objects: {objs}')
            actr = ActionInstance(coarse.get_action_by_name(course_action_name), objs, all_coarse_states[i].time_step)
            logging.info('Building zoomed system description...')
            zoomed_fine_res_description = zoom(all_coarse_states[i], all_coarse_states[i+1], actr, coarse, fine, True)
            fine_prog = zoomed_fine_res_description.to_sparc_program()
            fine_prog.save(os.path.join(os.environ['PLANNER_PATH'], 'sparc_planning/sparc_files/fine_temp.sp'))
            logging.info('Zooming complete')

            #run fine planner
            fine_plan = await plan(os.path.join(os.environ['PLANNER_PATH'], 'sparc_planning/sparc_files/fine_temp.sp'),
                                    max_length=fine_plan_length+FINE_ITERATION_LIMIT,
                                    min_length=fine_plan_length+FINE_ITERATION_MIN)
            #collect results
            fine_states, fine_actions = extract_states_from_answer_set(fine_plan[0],min_time_step=fine_plan_length)
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

        t_stop_fine = time.perf_counter()
        fine_plan_success = True

            # check final locations of obejcts and assembly status
            # fs8holds = [f for f in fine_state_fluents if not '-holds' in f]
            # fs8holdstrue = [f for f in fs8holds if 'true' in f]
            # fs8locations = [f for f in fs8holdstrue if 'loc_f' in f]
            # fs8assm = [f for f in fs8holdstrue if 'in_assembly_c' in f]

            # logging.debug(f'locations after fine plan: {fs8locations}')
            # logging.debug(f'assembly status after fine plan: {fs8assm}')
        
    finally:
        logging.info('\n=======RESULTS======\n')
        logging.info(f'Plan found with {len(abstract_actions)} Abstract Actions, took {t_stop_abs-t_start_abs:.03f} seconds')
        logging.info(f'Abstract Actions:\n {abstract_actions}\n\n')

        logging.info(f'Plan found with {len(all_coarse_actions)} Coarse Actions, took {t_stop_coarse-t_start_coarse:.03f} seconds')
        logging.info(f'Coarse Actions:\n {all_coarse_actions}\n\n')
        if fine_plan_success:
            logging.info(f'Fine Res plan found with {len(all_fine_actions)} Fine Actions, took {t_stop_fine-t_start_fine:.03f} seconds')
        else:
            # logging.critical(f'Fine Res plan failed after {len(all_fine_actions)} Fine Actions, took {t_stop_fine-t_start_fine:.03f} seconds')
            logging.critical(f'Fine Res plan failed after {len(all_fine_actions)} Fine Actions')
        logging.info(f'Fine Actions:\n {all_fine_actions}\n')

    

if __name__ == "__main__":
    asyncio.run(main())
