import os
import copy
from beam_domain_coarse import generate_coarse_beam_domain
from beam_domain_fine import generate_fine_beam_domain
from example_domains.example_mid_3 import generate_domain_setup
# from example_domains.example_latest import generate_domain_setup
import logging
import asyncio
from al_structures import ActionInstance, GoalDefinition
from planning import plan
from sparc_io import extract_states_from_answer_set
from zooming import remove_chars_from_last_number, zoom
import re
import time

from beam_assembly.beam_assembly_parser import load_beam_xml, load_assembly_xml
from beam_assembly.beam_to_sparc import create_sparc_data


MIN_FINE_PLAN_LENGTH = 0
MAX_FINE_PLAN_LENGTH = 40

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
    coarse = generate_coarse_beam_domain()
    # coarse.save_AL('/home/local/MTC_ORI_Collab/sparc_planning/action_lang_files/coarse_beam_AL.txt')
    #s = coarse.to_sparc_program()
    #s.save('/home/local/MTC_ORI_Collab/sparc_planning/sparc_files/beam_domain_coarse.sp')

    fine = generate_fine_beam_domain()
    # fine.save_AL('/home/local/MTC_ORI_Collab/sparc_planning/action_lang_files/fine_beam_AL.txt')

    beams = load_beam_xml(os.path.join(os.environ['PLANNER_PATH'], "example_beamset_latest.xml"))
    assem = load_assembly_xml(beams, os.path.join(os.environ['PLANNER_PATH'], "assembly_mid_3.xml"))
    
    # scene = assem.create_display_scene()
    # scene.show()

    # generate sparc planner data from xml
    coarse_sorts, xml_coarse_statics, fine_sorts, xml_fine_statics = create_sparc_data(assem)
    coarse_sort_dict = dict(zip([s.name for s in coarse_sorts], coarse_sorts))
    fine_sort_dict = dict(zip([s.name for s in fine_sorts], fine_sorts))

    logging.debug(f'Fine sorts: {[s.name for s in fine_sorts]}')

    # override coarse sorts with xml derived data
    for sort in coarse.sorts:
        if sort.name in coarse_sort_dict.keys():
            sort.instances = coarse_sort_dict[sort.name].instances
    # override fine sorts with xml derived data
    for sort in fine.sorts:
        if sort.name in fine_sort_dict.keys():
            sort.instances = fine_sort_dict[sort.name].instances

    # get domain setup data from python script
    coarse_fluents, example_coarse_statics, fine_fluents, example_fine_statics = generate_domain_setup()
    
    # combine with xml derived data
    coarse_statics = xml_coarse_statics + example_coarse_statics
    fine_statics = xml_fine_statics + example_fine_statics

    # update domain setup
    coarse.domain_setup = coarse_fluents + coarse_statics
    fine.domain_setup = fine_fluents + fine_statics

    # set a goal and create coarse sparc prog.
    fine.goal_description = [
        GoalDefinition('in_assembly_c',['b4'],True),
        GoalDefinition('fastened_c',['b7','b4','p1'],True),
        GoalDefinition('in_assembly_c',['b5'],True),
        GoalDefinition('fastened_c',['b7','b5','p2'],True),
        GoalDefinition('in_assembly_c',['b8'],True),
        GoalDefinition('fastened_c',['b5','b8','p3'],True),
        GoalDefinition('fastened_c',['b4','b8','p4'],True),
        GoalDefinition('in_assembly_c',['b1'],True),
        GoalDefinition('fastened_c',['b1','b8','p5'],True),
        GoalDefinition('fastened_c',['b1','b7','p6'],True),
        GoalDefinition('in_assembly_c',['b2'],True),
        GoalDefinition('fastened_c',['b2','b8','p7'],True),
        GoalDefinition('fastened_c',['b2','b7','p8'],True),
        ] # full coarse plan is 50 steps, a good lower bound heuristic is 6 to 7 steps per assembly goal
    #coarse_prog = coarse.to_sparc_program()
    #coarse_prog.save(os.path.join(os.environ['PLANNER_PATH'], 'sparc_planning/sparc_files/temp.sp'))

    # post full fine definition to file for debugging
    #fine_prog_test = fine.to_sparc_program()
    #fine_prog_test.save(os.path.join(os.environ['PLANNER_PATH'], 'sparc_planning/sparc_files/fine_unzoomed_temp.sp'))

    logging.warning('Planning may take some time, plans over 45 steps will likely take >100 seconds')
    t_start = time.perf_counter()
    #run fine planner
    fine_plan = await plan(os.path.join(os.environ['PLANNER_PATH'], 'sparc_planning/sparc_files/fine_unzoomed_temp.sp'),
                             max_length=MAX_FINE_PLAN_LENGTH, min_length=MIN_FINE_PLAN_LENGTH)

    #collect results
    fine_states, fine_actions = extract_states_from_answer_set(fine_plan[0]) #[0] just takes first answer set (valid plan), further work could explore which route to take
    t_stop = time.perf_counter()
    
    logging.info('\n=======RESULTS======\n')
    logging.info(f'Plan found with {len(fine_actions)} Fine Actions, took {t_stop-t_start:.03f} seconds')
    logging.info(f'Coarse Actions:\n {fine_actions}\n\n')
        

if __name__ == "__main__":
    asyncio.run(main())
