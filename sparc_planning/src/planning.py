from sparc_io import run_sparc, parse_sparc_ouput
import time
from typing import List
import re
import asyncio
import logging

def edit_numSteps(source:str, new_value:int):
    """alters the numSteps constant in a sparc file
    Args:
        source (str): sparc file location
        new_value (int): new numSteps value
    """
    with open(source, 'r') as file:
        file_string = file.read()
    file_string = (re.sub('#const numSteps = \d+.', f'#const numSteps = {new_value}.', file_string))
    with open(source, 'w') as file:
        file.write(file_string)

async def run_sparc_coroutine(sparc_file_loc):
    run_sparc(sparc_file_loc)

async def plan(
        sparc_file_location:str = 'temp.sp',
        sparc_output_location:str = 'sparc.out',
        min_length:int = 1,
        max_length:int = 50,
        max_plan_cycle_time:int = 10,
        ) -> List[str]:
    start_time = time.perf_counter()
    logging.info('Planning started...')
    for i in range(min_length,max_length+1,1):
        logging.info(f'Running sparc with numSteps = {i}')
        t1 = time.perf_counter()
        # increment planning horizon
        edit_numSteps(sparc_file_location, i)
        # try to get answer sets (with timeout)
        task = run_sparc_coroutine(sparc_file_location)
        try:
            await asyncio.wait_for(task, timeout = max_plan_cycle_time)
        except asyncio.TimeoutError:
            logging.error(f'Sparc failed to return in less than {max_plan_cycle_time} seconds')
        # test if output contains 1 or more answer set
        res = parse_sparc_ouput(sparc_output_location)
        # crude timer
        t2 = time.perf_counter()
        logging.info(f'Cycle time: {t2-t1:10.4f} seconds')
        if len(res)>0:
            logging.info(f'Plan found in: {t2 - start_time:10.4f} seconds, with length {i} steps')
            return res
    logging.error(f'Failed to find solution in {max_length} steps')
    raise ValueError

#?test
#asyncio.run(plan('/home/local/MTC_ORI_Collab/sparc_planning/sparc_files/beam_domain_fine.sp', max_length=7, min_length=1))
