import copy
from typing import List, Tuple
import re
from al_structures import Action, ActionLangSysDesc, FuncType, GoalDefinition, Property, Relation, SortType, BasicSort, SuperSort, Func, ActionInstance
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

def zoom(s1:SparcState, s2:SparcState, aH:ActionInstance, DH:ActionLangSysDesc, DLR:ActionLangSysDesc, location_restriction:bool=False):
    """returns a zoomed system description for the transition aH
    Args:
        s1 (SparcState): coarse resolution state (answer set) before transition
        s2 (SparcState): coarse resolution state (answer set) after transition
        aH (ActionInstance): coarse resolution action instance causing the transition
        DH (ActionLangSysDesc): coarse resolution domain description
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
    logging.debug(f'relObConH from action: {relObConH}')
    
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
        func = ret.group()[6:-1] #TODO this doesnt work for 'val' style
        # extract object_instances from funcs using string splitting
        # expects format 'foo(x1,x2,...xn)' with objects x1,x2,..xn
        for f in re.split('\(|\)|\,', func)[1:-1]:
            relObConH.add(f)
    
    logging.debug(f'relObConH + from differences between s1 and s2 fluents: {relObConH}')
        
    #13.3 (if body B of an executability condition of aH contains an occurrence
    #  of a term f (x1, ... , xn) and f(x1, ... , xn) = y ∈ σ1
    #  then x1, ..., xn, y are in relObConH(T))
    # relevant object_instances from executability conditions of aH
    # Example might be that course resolution is to pickup a book in kitchen,
    # ah = pickup(rob0,book). where state1 contains location(book, kitchen)
    # At the fine resolution we need to know about areas in the kitchen 
    # eg. grid cells refined versions of #place = kitchen.
    # Which we don't know about at high level.
    aoin = aH.object_constants
    for e in aH.action.executability_conditions:
        for i in range(len(e.conditions)):
            # want to check if state 1 contains func(objects including aH objects)
            # if state one domain setup contains any gound instance e.g. loc_c(rob0, \w+)
            # then the other objects in that gound instance are relevant  
            for i in range(len(e.conditions)):
                cond = e.conditions[i]
                if isinstance(cond,Property): continue # ignore properties
                object_monikers = e.condition_object_instance_names[i]
                y = e.condition_values[i]
                object_sorts = [e.object_instances[o] for o in object_monikers]
                for i in range(len(object_sorts)):
                    s = object_sorts[i]
                    sort_obs = DH.get_sort_objects(s)
                    # look for instances with same name as our action objects from aH
                    for act_ob in aoin:
                        if act_ob in sort_obs: 
                            query = ['\w+']*len(object_sorts)
                            query[i] = act_ob
                            if cond.func_type.value == FuncType.FLUENT.value:
                                # find functions of the form holds(condition(x1,..,x2),bool
                                res = re.findall("(?<!-)holds\("+cond.name+"\("+'\,'.join(query)+f"\)\,{str(y).lower()}", ' '.join(s1flu)) 
                                res = [r[6:] for r in res] # remove "holds("
                            if cond.func_type.value == FuncType.STATIC.value:
                                term = '(?<!-)'
                                if y == False: term = "-"
                                res = re.findall(term+f"{cond.name}\("+'\,'.join(query)+'\)', ' '.join(s1.statics)) 
                # parse relevant objects if found
                if len(res) == 0 : continue
                for func in res:
                    for f in re.split('\(|\)|\,', func)[1:-1]:
                        if f == '': continue
                        relObConH.add(f)

    logging.debug(f'Relevant objects (coarse resolution): {relObConH}')

    # identify any relevant objects and components in fine resolution
    relevant_fine_objects = copy.deepcopy(relObConH)
    for s in DLR.domain_setup:
        # find component relations (expected format 'component(coarse, fine).')
        if 'component' in s:
            if s[0] == '%': continue # skip comment lines
            #logging.debug(s)
            obs = re.split('\(|\)|\,', s)[1:-1]
            # check that object is relevant at coarse level
            if obs[0] in relObConH:
                # add fine res object to set
                relevant_fine_objects.add(obs[1])

    logging.debug(f'Relevant objects (fine resolution): {relevant_fine_objects}')

    # STEP 2: extract relevant fine system description DLRT
    sorts_DLRT = set()
    for sort in DLR.sorts:
        # check if this is a basic sort
        if sort.sort_type == SortType.BASIC:
            # finds objects which are in both relObConH and the sort from DLR
            objects = set.intersection(relevant_fine_objects,{*sort.instances})
            if len(objects)>0:
                # defines new sort with the same name but only relevant objects
                sorts_DLRT.add(BasicSort(sort.name,objects))
  
    # find relevant parent sorts (structure: #sort = #subsort + #subsort2.)
    # parent sorts do not have sort in function f(#sort) format so we remove any with brackets in
    # needs to loop incase new sort is also a subsort, unlikely to loop more than 3 times
    logging.debug('Looping through sort heirarchy to find all relevant super sorts')
    while True:
        new_sorts = set()
        # cycle through sorts of DLR
        for parent_sort in DLR.sorts:
            if parent_sort.sort_type == SortType.SET:
                relevant_subsorts = set.intersection({*[f'#{s.name}' for s in sorts_DLRT]}, {*parent_sort.instances})
                if len(relevant_subsorts) > 0:
                    subsorts = [s for s in sorts_DLRT if (f'#{s.name}' in relevant_subsorts)]
                    # if this sort hasnt already been added then cache it
                    if parent_sort.name not in [s.name for s in sorts_DLRT]:
                        new_sorts.add(SuperSort(parent_sort.name,subsorts))
                        #if this sort has been added check that all relevant sub sorts are caught, if not replace with new version
                    else:
                        for i in sorts_DLRT:
                            if i.name == parent_sort.name: 
                                dlrt_sort = i
                        if not set([s.name for s in dlrt_sort.subsorts]) == set([s.name for s in subsorts]):
                            sorts_DLRT.remove(dlrt_sort)
                            new_sorts.add(SuperSort(parent_sort.name,subsorts))
                            logging.debug(f'Updating sort: {parent_sort.name} with subsorts: {[s.name for s in subsorts]}')
        # add new sorts to our search list and continue
        sorts_DLRT = sorts_DLRT.union(new_sorts)
        logging.debug(f'Added sorts: {[s.name for s in new_sorts]}')
        # stop looping when no new subsorts are found
        if len(new_sorts) == 0:
            break
        
    logging.debug(f'Relevant Sorts (fine res): {[s.name for s in sorts_DLRT]}')

    if location_restriction:
        # concept here is that we only care about relevant assembly locaitons, i.e. if we are inserting pin1,
        # we dont care about the locations for assembling beam15 which would otherwise be identified as locations
        # within the assembly area in the default zooming operation.
        #todo test impact on planning speed.
        target_locations,refined_target_locs = [],[]
        approach_locations,refined_approach_locs = [],[]
        prerot_locations,refined_prerot_locs = [],[]
        through_locations,refined_through_locs = [],[]
        # grab instances of relevant sorts
        relevant_things = []
        for sort in sorts_DLRT:
            # thing is a super sort with subsorts containing handlable objects
            if sort.name == 'thing':
                for subsort in sort.subsorts:
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
        assembly_locations = target_locations + approach_locations + prerot_locations + through_locations
        relevant_fine_objects = relevant_fine_objects - {*assembly_locations}
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
        relevant_fine_objects = relevant_fine_objects.union({*refined_assembly_locations})
        logging.debug(f'Relevant objects (post location reduction): {relevant_fine_objects}')
    
    # The domain attributes and actions of ΣLR(T) are those of ΣLR restricted to the basic sorts of ΣLR(T)
    sorts_DLRT_names = [s.name for s in sorts_DLRT]

    new_actions = []
    for act in DLR.actions:
        if {*[s.name for s in act.action_def.sorts]}.issubset(sorts_DLRT_names):
            # remove executability conditions where cond = true #unknown_sort(A)
            # remove actions where exec cond = false #unknown_sort(A) (not possible)
            nes = set()
            action_possible = True
            exec_conditions = []
            for execut in act.executability_conditions:
                remove_exec_cond = False
                if not execut.conditions: 
                    exec_conditions.append(execut)
                    continue
                for _c in range(len(execut.conditions)):
                    if not action_possible: continue # skip ahead if action will be removed anyway
                    if remove_exec_cond: continue # skip ahead if this exec condition will be removed anyway
                    if execut.condition_values[_c] == True:
                        if isinstance(execut.conditions[_c],Property):
                            #check if is a #sort(A) relation
                            if execut.conditions[_c].relation.value == Relation.IS_OF_SORT.value:
                                if execut.conditions[_c].object1 not in sorts_DLRT_names:
                                    remove_exec_cond = True
                        else:
                            for sort_ in  execut.conditions[_c].sorts:
                                if not sort_.name in sorts_DLRT_names:
                                    remove_exec_cond = True
                    else: # condition_value == False:
                        if isinstance(execut.conditions[_c],Property):
                            #check if is a #sort(A) relation
                            if execut.conditions[_c].relation.value == Relation.IS_OF_SORT.value:
                                if execut.conditions[_c].object1 not in sorts_DLRT_names:
                                    action_possible = False
                        else:
                            for sort_ in  execut.conditions[_c].sorts:
                                if not sort.name in sorts_DLRT_names:
                                    action_possible = False
                if not remove_exec_cond: 
                    exec_conditions.append(execut)         
            if not action_possible: 
                continue
            new_act_cls = []
            # add empty basic sort where causal law always holds, cond if #unknown_sort(A) = False, 
            # remove causal laws which cannot hold,  cond = #unknown_sort(A) = True,
            for cl in act.causal_laws:
                remove_casual_law = False
                if not cl.conditions:
                    new_act_cls.append(cl)
                    continue
                for _c in range(len(cl.conditions)):
                    if cl.condition_values[_c] == True:
                        if isinstance(cl.conditions[_c],Property):
                            #check if is a #sort(A) relation
                            if cl.conditions[_c].relation.value == Relation.IS_OF_SORT.value:
                                if cl.conditions[_c].object1 not in sorts_DLRT_names:
                                    remove_casual_law = True
                        else:
                            for sort_ in  cl.conditions[_c].sorts:
                                if not sort_.name in sorts_DLRT_names:
                                    remove_casual_law = True
                    else: # condition_value == False:
                        if isinstance(cl.conditions[_c],Property):
                            #check if is a #sort(A) relation
                            if cl.conditions[_c].relation.value == Relation.IS_OF_SORT.value:
                                if cl.conditions[_c].object1 not in sorts_DLRT_names:
                                    # add empty sort
                                    nes.add(cl.conditions[_c].object1)
                        else:
                            for sort_ in  cl.conditions[_c].sorts:
                                if not sort_.name in sorts_DLRT_names:
                                    # add empty sort
                                    nes.add(sort_.name)
                if not remove_casual_law: 
                    new_act_cls.append(cl)

            new_actions.append(Action(action_def=act.action_def,
                                        executability_conditions=exec_conditions,
                                        causal_laws=new_act_cls))
            new_empty_sorts = {*[BasicSort(nom, []) for nom in nes]}
            sorts_DLRT = sorts_DLRT.union(new_empty_sorts)
    #update sort names list
    sorts_DLRT_names = [s.name for s in sorts_DLRT]

    new_inertial_fluents = []
    for ifluent in DLR.inertial_fluents:
        if {*[s.name for s in ifluent.sorts]}.issubset(sorts_DLRT_names):
            new_inertial_fluents.append(ifluent)

    new_defined_fluents = []
    if DLR.defined_fluents:
        for dfluent in DLR.defined_fluents:
            if {*[s.name for s in dfluent.sorts]}.issubset(sorts_DLRT_names):
                new_defined_fluents.append(dfluent)

    # the axioms of DLR(T) are restrictions of axioms of DLR to ΣLR(T).
    new_state_contraints = []
    for stcon in DLR.state_constraints:
        if {*[s.name for s in stcon.object_instances.values()]}.issubset(sorts_DLRT_names):
            new_state_contraints.append(stcon)

    new_statics = []
    for stat in DLR.statics:
        if {*[s.name for s in stat.sorts]}.issubset(sorts_DLRT_names):
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
        obs = set()
        func = s
        ret = re.search('holds\(.+\(.+\),', s)
        if ret:
            func = ret.group()[6:-1]
        for f in re.split('\(|\)|\,', func)[1:-1]:
            obs.add(f)
        if obs.issubset(relevant_fine_objects):
            new_domain_setup.append(s)
    
    # goals - should be drawn from coarse state after desired transition (s2),
    # i.e., goal should be to obtain the same state through one or more fine res transitions
    # therefore fine res goal should be s2.fluents not in s1
    new_goal_statements = {*s2flu}-{*s1flu}
    new_goals = []
    # s2flu are statments of the format: "holds(func(x1,x2..xn),bool," or "-holds(...)"
    for statement in new_goal_statements:
        b = False
        if re.search('true', statement): b = True
        ret = re.search('(?<!-)holds\(.+\(.+\),', statement)
        if ret:
            func = ret.group()[6:-1]
            # extract object_instances from funcs using string splitting
            # expects format 'foo(x1,x2,...xn)' with objects x1,x2,..xn
            data = re.split('\(|\)|\,', func)[:-1]
            new_goals.append(GoalDefinition(data[0], data[1:], b))

    # sort ordering to prevent errors
    basic_sorts = [s for s in sorts_DLRT if s.sort_type==SortType.BASIC]
    super_sorts = [s for s in sorts_DLRT if s.sort_type==SortType.SET]

    # super_sorts need to be ordered so that any sort in the subsorts list comes before the super_sort
    spin = 0
    while True:
        for i in range(len(super_sorts)):
            sort = super_sorts[i]
            sub_sort_names = [s[1:] for s in sort.instances]
            super_sort_names = [s.name for s in super_sorts]
            sort_ind = super_sort_names.index(sort.name)
            sub_sort_inds = []
            for s in sub_sort_names:
                if s in super_sort_names:
                    sub_sort_inds.append(super_sort_names.index(s))
            if any([s>sort_ind for s in sub_sort_inds]):
                # move to end
                super_sorts.remove(sort)
                super_sorts.append(sort)
                spin += 1
        if spin == 0: break
        spin = 0

    ordered_sorts = basic_sorts+super_sorts

    # create new fine resolution system description
    return ActionLangSysDesc(
            sorts = ordered_sorts,
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
            start_step=DLR.start_step
        )
    
    