import copy
from typing import List, Tuple
import re
from al_structures import ActionLangSysDesc, SortType, BasicSort, SuperSort, Func, ActionInstance
from sparc_io import SparcState
import re
import logging

def remove_chars_from_last_number(original_string:str) -> str:
    """helper function removes the end of a string after the last number.
    This is useful for removing timestamps

    e.g. if string is 'holds(in_asssmebly(b1), true, 20).'
    result = 'holds(in_asssmebly(b1), true, '

    Args:
        original_string (str): input
    Returns:
        str: truncated string
    """
    n = re.search('\d+',original_string[::-1]).span()[1]
    return original_string[:-n]

def extract_state_transition(states:List[SparcState], occurs:List[str], DH:ActionLangSysDesc, step:int = 0) -> Tuple[SparcState, SparcState, ActionInstance]:
    """seperates state data for before and after action and creates ActionInstance object for transition aH.
    assumes single action at each timestep.

    Args:
        states (List[SparcState]): state data from answer set
        occurs (List[str]): action predicitons from answer set
        DH (ActionLangSysDesc): action language (at action resolution) definition of system
        step (int, optional): which transition in occurs if len(occurs)>1. Defaults to 0.
    Returns:
        Tuple[SparcState, SparcState, ActionInstance]: zoom inputs s0,s1,aH
    """
    occ = [oc for oc in occurs if int(oc[-2])==step] 
    if len(occ) == 0:
        raise ValueError(f'no action in set {occurs} for time_step {step}')
    # assumes single action at each timestep (occ[0] because list has a single element)
    action_full_text = re.search('occurs\(.+\(.+\),', occ[0]).group()[7:-1]
    action_name, *object_constants = re.split('\(|\)|\,', action_full_text)[:-1]
    for ac in DH.actions:
        if ac.action_def.name == action_name:
            aH = ActionInstance(ac,object_constants,step)
    
    state0 = None
    state1 = None
    #find state data
    for state in states:
        if state.time_step == step:
            state0 = state
        if state.time_step == step+1:
            state1 = state
    if isinstance(state0, type(None)) or isinstance(state1, type(None)): 
        raise ValueError(f'Could not find state data for both timestep {step} and {step+1}')

    return state0, state1, aH

def zoom(s1:SparcState, s2:SparcState, aH:ActionInstance, DLR:ActionLangSysDesc, location_restriction:bool=False):
    """returns a zoomed system description for the transition aH
    Args:
        s1 (SparcState): coarse resolution state (answer set) before transition
        s2 (SparcState): coarse resolution state (answer set) after transition
        aH (ActionInstance): coarse resolution action instance causing the transition 
        DLR (ActionLangSysDesc): fine resolution domain description which
                is a refinement of the coarse resolution domain DH.
        location_restriction(bool): toggles additional scope reduction on location
                objects specific to the robotic assembly domain system. default = False
    Returns:
        ActionLangSysDesc: DLR(T) zoomed fine resolution system desc.
    """
    # STEP 1: Define relevant object constants
    # 13.1 start with set of object constants from aH
    relObConH = {*aH.object_constants}
    
    # states strings are timestamped so this needs to be fixed for comparison
    s1flu = [remove_chars_from_last_number(f1) for f1 in s1.fluents]
    s2flu = [remove_chars_from_last_number(f2) for f2 in s2.fluents]
    # statics dont change so can be ignored  
    # 13.2
    # functions in s1 or s2 but not both
    funcs = {*s1flu}.difference({*s2flu}).union({*s2flu}.difference({*s1flu}))
    for func in funcs:
        # extract functions from holds(func(x1,x2..xn),step) statements
        ret = re.search('holds\(.+\(.+\),', func)
        if not ret:
            ret = re.search('val\(.+\(.+\),', func)
        func = ret.group()[6:-1]
        # extract object_instances from funcs using string splitting
        # expects format 'foo(x1,x2,...xn)' with objects x1,x2,..xn
        for f in re.split('\(|\)|\,', func)[1:-1]:
            relObConH.add(f)
        
    #13.3
    # relevant object_instances from executability conditions of aH
    # Example might be that course resolution is to pickup a book in kitchen,
    # ah = pickup(rob0,book). where state1 contains location(book, kitchen)
    # At the fine resolution we need to know about areas in the kitchen 
    # eg. grid cells refined versions of #place = kitchen.
    # Which we don't know about at high level.
    aoin = aH.ActionDescription.action_object_instance_names
    for e in aH.ActionDescription.excutability_conditions:
        e_statements = []
        for i in len(e.conditions):
            cond = e.conditions[i]
            if not isinstance(cond, Func):
                continue
            # want to check if state 1 contains func(objects including aH objects)
            inst_names = copy.deepcopy(e.condition_object_instance_names[i])
            # replace unknown object names with '\w+' to accept any word in regular expression serch
            for j in len(inst_names):
                if inst_names[j] not in aoin:
                    inst_names[j] = '\w+'
            # create search string and search state1 adding any results to e_statements
            s = f"{cond.name}\({','.join(inst_names)}\)"
            e_statements += re.findall(s, s1)
        # extract object_instances from funcs using string splitting
        # expects format 'foo(x1,x2,...xn)' with objects x1,x2,..xn
        for func in e_statements:
            for f in re.split('\(|\)|\,', func)[1:-1]:
                relObConH.add(f)
                
    logging.info('Relevant objects (coarse resolution): ', relObConH)

    # identify any relevant objects and components in fine resolution
    relevant_fine_objects = copy.deepcopy(relObConH)
    for s in DLR.domain_setup:
        # find component relations (expected format 'component(coarse, fine).')
        if 'component' in s:
            obs = re.split('\(|\)|\,', s)[1:-1]
            # check that object is relevant at coarse level
            if obs[0] in relObConH:
                # add fine res object to set
                relevant_fine_objects.add(obs[1])

    logging.info('Relevant objects (fine resolution): ', relevant_fine_objects)

    # STEP 2: extract relevant fine system description DLRT
    sorts_DLRT = set()
    for sort in DLR.sorts:
        # check if this is a basic sort
        if sort.sort_type == SortType.BASIC:
            # finds objects which are in both relObConH and the sort from DLR
            objects = set.intersection(relObConH,{*sort.instances})
            if len(objects)>0:
                # defines new sort with the same name but only relevant objects
                sorts_DLRT.add(BasicSort(sort.name,objects))
    
    # find relevant parent sorts (structure: #sort = #subsort + #subsort2.)
    # parent sorts do not have sort in function f(#sort) format so we remove any with brackets in
    # needs to loop incase new sort is also a subsort, unlikely to loop more than 3 times
    while True:
        new_sorts = set()
        # cycle through sorts of DLR
        for parent_sort in DLR.sorts:
            if parent_sort.sort_type == SortType.SET:
                relevant_subsorts = set.intersection({*[f'#{s.name}' for s in sorts_DLRT]}, {*parent_sort.instances})
                if len(relevant_subsorts) > 0:
                    # if this sort hasnt already been added then cache it
                    if parent_sort not in sorts_DLRT:
                        subsorts = [s for s in sorts_DLRT if (s.name in relevant_subsorts)]
                        new_sorts.add(SuperSort(parent_sort.name,subsorts))
        # stop looping when no new subsorts are found
        if len(new_sorts) == 0:
            break
        # otherwise add them to our search list and continue
        sorts_DLRT = sorts_DLRT.union(new_sorts)
    
    logging.info('Relevant Sorts: ', [s.name for s in sorts_DLRT])

    if location_restriction:
        refined_target_locs = []
        refined_approach_locs = []
        refined_prerot_locs = []
        refined_through_locs = []
        # grab instances of relevant sorts
        relevant_things = []
        for sort in sorts_DLRT:
            # thing is a super sort with subsorts containing handlable objects
            if sort.name == 'thing':
                for subsort in sort:
                    relevant_things += subsort.instances
            if sort.name == 'target_locations':
                target_locations = sort.instances
            if sort.name == 'approach_locations':
                approach_locations = sort.instances
            if sort.name == 'prerot_location':
                prerot_locations = sort.instances
            if sort.name == 'through_location':
                through_locations = sort.instances
        # temporarily remove locations from relevant_fine_objects
        assembly_locations = target_locations + approach_loc + prerot_locations + through_locations
        relevant_fine_objects -= {*assembly_locations}
        # for each thing
        for thing in relevant_things:
            # check for relevant locations
            for target_loc in target_locations:
                if f'assem_target_loc({thing}, {target_loc}).' in DLR.domain_setup:
                    refined_target_locs.append(target_loc)
            for approach_loc in approach_locations:
                if f'assem_approach_loc({thing}, {approach_loc}).' in DLR.domain_setup:
                    refined_approach_locs.append(approach_loc)
            for pr_loc in prerot_locations:
                if f'beam_prerotate_loc({thing}, {pr_loc}).' in DLR.domain_setup:
                    refined_prerot_locs.append(pr_loc)
            for th_loc in through_locations:
                if f'beam_through_loc({thing}, {th_loc}).' in DLR.domain_setup:
                    refined_through_locs.append(th_loc)
        # update sort instances of DLRT
        for sort in sorts_DLRT:
            if sort.name == 'target_locations':
                sort.instances = refined_target_locs
            if sort.name == 'approach_locations':
                sort.instances = refined_approach_locs
            if sort.name == 'prerot_location':
                sort.instances = refined_prerot_locs
            if sort.name == 'through_location':
                sort.instances = refined_through_locs
        # update relevant_fine_objects to put back good locations
        refined_assembly_locations = refined_target_locs + refined_approach_locs + refined_prerot_locs + refined_through_locs
        relevant_fine_objects += {*refined_assembly_locations}
        logging.info('Relevant objects (post location reduction): ', relevant_fine_objects)
    
    # The domain attributes and actions of ΣLR(T) are those of ΣLR restricted to the basic sorts of ΣLR(T)
    new_inertial_fluents = []
    for ifluent in DLR.inertial_fluents:
        if {*ifluent.sorts}.issubset(sorts_DLRT):
            new_inertial_fluents.append(ifluent)

    new_defined_fluents = []
    for dfluent in DLR.defined_fluents:
        if {*dfluent.sorts}.issubset(sorts_DLRT):
            new_defined_fluents.append(dfluent)

    new_actions = []
    for act in DLR.actions:
        if {*act.sorts}.issubset(sorts_DLRT):
            new_actions.append(act)

    # the axioms of DLR(T) are restrictions of axioms of DLR to ΣLR(T).
    new_state_contraints = []
    for stcon in DLR.state_constraints:
        if {*stcon.object_instances.values()}.issubset(sorts_DLRT):
            new_state_contraints.append(stcon)

    new_statics = []
    for stat in DLR.statics:
        if {*stat.sorts}.issubset(sorts_DLRT):
            new_statics.append(stat)

    # we also need to refine goal and domain details as otherwise objects referenced
    # may not be defined in sparc file leading to failures in planning. For example
    # if course goal is 
    # 'goal :- location(block1,kitchen), location(block2,study).'
    # We don't care about block1 or the kitchen whilst moving block2.
    
    new_domain_setup = []
    # domain setup is made up of strings refering to specific objects (history) 
    for s in DLR.domain_setup:
        # check if all objects are relevant to this transistion
        obs = {}
        func = s
        ret = re.search('holds\(.+\(.+\),', s)
        if ret:
            func = ret.group()[6:-1]
        for f in re.split('\(|\)|\,', func)[1:-1]:
            obs.add(f)
        if obs.issubset(relevant_fine_objects):
            new_domain_setup.append(s)
    
    # goals
    new_goals = []
    for g in DLR.goal_description:
        if {*g.object_instances}.issubset(relevant_fine_objects):
            new_goals.append(g)

    # create new fine resolution system description
    return ActionLangSysDesc(
            sorts = sorts_DLRT,
            inertial_fluents = new_inertial_fluents,
            actions = new_actions, # List[Action]
            domain_setup = new_domain_setup , # List[str]
            goal_description = new_goals , # List[GoalDescription]
            defined_fluents  = new_defined_fluents, # Optional[List[Func]]
            statics  = new_statics, # Optional[List[Func]]
            state_constraints = new_state_contraints, # Optional[List[StateConstraint]]
            constants = DLR.constants ,# constants remain the same  : Optional[List[Constant]]
            display_hints = None,
            planning_steps = 1,
        )
    
    