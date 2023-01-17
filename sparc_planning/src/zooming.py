import copy
from sparc_io import SparcProg
from typing import List
from dataclasses import dataclass
import re
from al_structures import ActionLangSysDesc, SortType, BasicSort, SuperSort, Func, ActionInstance

# can find each action from answer set by displaying
# occurs. and extract by timestep example result:
# occurs(	add(d2)	, 1	)	.
# occurs(	add(d3)	, 0	)	.
# occurs(	add(d4)	, 2	)	.

# can find state of fluents in answer sets by inspecting 
# display val. and extracting by timestep e.g.:
# val(	in_assembly(d1)	, true	, 0	)	.
# val(	in_assembly(d1)	, true	, 1	)	.
# val(	in_assembly(d1)	, true	, 2	)	.
# val(	in_assembly(d1)	, true	, 3	)	.
# val(	in_assembly(d2)	, true	, 2	)	.
# val(	in_assembly(d2)	, true	, 3	)	.
# val(	in_assembly(d3)	, true	, 1	)	.
# val(	in_assembly(d3)	, true	, 2	)	.
# val(	in_assembly(d3)	, true	, 3	)	.
# val(	in_assembly(d4)	, true	, 3	)	.
# val(	supported(d1)	, true	, 1	)	.
# val(	supported(d4)	, true	, 1	)	.
# val(	supported(d1)	, true	, 2	)	.
# val(	supported(d4)	, true	, 2	)	.
# val(	supported(d1)	, true	, 3	)	.
# val(	supported(d4)	, true	, 3	)	.
# val(	supported(d2)	, true	, 0	)	.
# val(	supported(d3)	, true	, 0	)	.
# val(	supported(d2)	, true	, 1	)	.
# val(	supported(d3)	, true	, 1	)	.
# val(	supported(d2)	, true	, 2	)	.
# val(	supported(d3)	, true	, 2	)	.
# val(	supported(d2)	, true	, 3	)	.
# val(	supported(d3)	, true	, 3	)	.

# need to add statics also?

# state1 at time 0
s1 =[
    'in_assembly(d1)',
    'supported(d2)',
    'supported(d3)',
    ]

# state2 at time 1
s2 =[
    'in_assembly(d1)',
    'in_assembly(d3)',
    'supported(d1)',
    'supported(d2)',
    'supported(d3)',
    'supported(d4)',
    #'basic_bollard(b1,b2)'
    ]



def zoom(s1:List[str], s2:List[str], aH:ActionInstance, DH:ActionLangSysDesc, ):
    # STEP 1: Define relevant object constants
    # 13.1 start with set of object constants from aH
    relObConH = {*aH.object_constants}
    
    # 13.2
    # functions in s1 or s2 but not both
    funcs = {*s1}.difference({*s2}).union({*s2}.difference({*s1}))
    for func in funcs:
        # extract functions from holds(func(x1,x2..xn),step) statements
        ret = re.search('holds\(.+\(.+\),', func)
        if ret:
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
                
    print('Relevant objects: ', relObConH)
    
    # STEP 2: extract relevant system description DHT
    sorts_DHT = set()
    for sort in DH.sorts:
        # check if this is a basic sort
        if sort.sort_type == SortType.BASIC:
            # finds objects which are in both relObConH and the sort from DH
            objects = set.intersection(relObConH,{*sort.instances})
            if len(objects)>0:
                # defines new sort with the same name but only relevant objects
                sorts_DHT.add(BasicSort(sort.name,objects))
    
    # find relevant parent sorts (structure: #sort = #subsort + #subsort2.)
    # parent sorts do not have sort in function f(#sort) format so we remove any with brackets in
    # needs to loop incase new sort is also a subsort, unlikely to loop more than 3 times
    while True:
        new_sorts = set()
        # cycle through sorts of DH
        for parent_sort in DH.sorts:
            if parent_sort.sort_type == SortType.SET:
                relevant_subsorts = set.intersection({*[f'#{s.name}' for s in sorts_DHT]}, {*parent_sort.instances})
                if len(relevant_subsorts) > 0:
                    # if this sort hasnt already been added then cache it
                    if parent_sort not in sorts_DHT:
                        subsorts = [s for s in sorts_DHT if (s.name in relevant_subsorts)]
                        new_sorts.add(SuperSort(parent_sort.name,subsorts))
        # stop looping when no new subsorts are found
        if len(new_sorts) == 0:
            break
        # otherwise add them to our search list and continue
        sorts_DHT = sorts_DHT.union(new_sorts)
    
    print('Relevant Sorts: ', sorts_DHT)
    
    # The domain attributes and actions of ΣH(T) are those of ΣH restricted to the basic sorts of ΣH(T)
    new_inertial_fluents = []
    for ifluent in DH.inertial_fluents:
        if {*ifluent.sorts}.issubset(sorts_DHT):
            new_inertial_fluents.append(ifluent)

    new_defined_fluents = []
    for dfluent in DH.defined_fluents:
        if {*dfluent.sorts}.issubset(sorts_DHT):
            new_defined_fluents.append(dfluent)

    new_actions = []
    for act in DH.actions:
        if {*act.sorts}.issubset(sorts_DHT):
            new_actions.append(act)

    # the axioms of DH(T) are restrictions of axioms of DH to ΣH(T).
    new_state_contraints = []
    for stcon in DH.state_constraints:
        if {*stcon.object_instances.values()}.issubset(sorts_DHT):
            new_state_contraints.append(stcon)

    new_statics = []
    for stat in DH.statics:
        if {*stat.sorts}.issubset(sorts_DHT):
            new_statics.append(stat)

    # we also need to refine goal and domain details as otherwise objects referenced
    # may not be defined in sparc file leading to failures in planning. For example
    # if course goal is location(block1,kitchen), location(block2,study). We don't 
    # care about block1 or the kitchen whilst moving block2.
    
    new_domain_setup = []
    # domain setup is made up of strings refering to specific objects (history) 
    for s in DH.domain_setup:
        # check if all objects are relevant to this transistion
        obs = {}
        func = s
        ret = re.search('holds\(.+\(.+\),', s)
        if ret:
            func = ret.group()[6:-1]
        for f in re.split('\(|\)|\,', func)[1:-1]:
            obs.add(f)
        if obs.issubset(relObConH):
            new_domain_setup.append(s)
    
    # goals
    new_goals = []
    for g in DH.goal_description:
        if {*g.object_instances}.issubset(relObConH):
            new_goals.append(g)

    # create new course resolution system description
    DHT = ActionLangSysDesc(
            sorts = sorts_DHT,
            inertial_fluents = new_inertial_fluents,
            actions = new_actions, # List[Action]
            domain_setup = new_domain_setup , # List[str]
            goal_description = new_goals , # List[GoalDescription]
            defined_fluents  = new_defined_fluents, # Optional[List[Func]]
            statics  = new_statics, # Optional[List[Func]]
            state_constraints = new_state_contraints, # Optional[List[StateConstraint]]
            constants = DH.constants ,# constants remain the same  : Optional[List[Constant]]
            display_hints = None,
            planning_steps = 1,
        )

    #!
    #TODO
    #!
    #! extract fine resolution description for this signature
    
        
dh = SparcProg.load(r'E:\linux_folder_backup_22-12-22\MTCORICollab\beam_domain_course.sp')        
zoom(s1,s2,a, dh)
    
    
    