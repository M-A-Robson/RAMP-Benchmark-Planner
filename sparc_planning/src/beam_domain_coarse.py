import os
from sparc_planning.src.al_structures import *

SAVE_DIR = os.path.join(os.environ['PLANNER_PATH'], '/sparc_planning/sparc_files/')

def generate_coarse_beam_domain():

    #!sorts
    robot = BasicSort('robot', ['rob0'])
    #add beam and pin sorts
    beam = BasicSort('beam',['b1','b2','b3','b4'])
    pin = BasicSort('pin',['p1','p2','p3','p4'])
    thing = SuperSort('thing', [beam,pin])
    ob = SuperSort('object', [robot, thing])
    place = BasicSort('place_c', ['input_area', 'intermediate_area', 'assembly_area'])
    #cannot pick using vacuum so we can eliminate grasp mode changing
    #grasp_mode = BasicSort('grasp_mode', ['dexterous', 'vacuum'])

    #!statics
    next_to = Func('next_to_c', [place,place], FuncType.STATIC)
    # add beam domain statics
    fits_into = Func('fits_into_c', [beam,beam], FuncType.STATIC)
    fits_through = Func('fits_through_c',[beam,beam],FuncType.STATIC)
    is_capped_by = Func('is_capped_by',[beam,beam,beam],FuncType.STATIC)
    base = Func('base',[beam],FuncType.STATIC)

    statics = [next_to,fits_into,fits_through,is_capped_by,base]

    #!fluents
    in_hand = Func('in_hand_c', [robot,thing], FuncType.FLUENT)
    location = Func('loc_c', [ob,place], FuncType.FLUENT)
    # add beam domain fluents
    in_assembly = Func('in_assembly_c',[beam],FuncType.FLUENT)
    supported = Func('supported_c',[beam], FuncType.FLUENT)
    fastened = Func('fastened_c',[beam,beam,pin], FuncType.FLUENT)
    misaligned = Func('misaligned_c', [beam], FuncType.FLUENT)
    can_fasten = Func('can_fasten_c', [beam,beam], FuncType.FLUENT)

    fluents = [in_hand,location,in_assembly,supported,
               fastened, misaligned, can_fasten]

    #!state constraints
    state_constraints=[]
    #CWA on base
    state_constraints.append(StateConstraint(
        head= base,
        head_value= False,
        head_object_instance_names=['B2'],
        object_instances={'B1':beam,'B2':beam},
        conditions=[base, Property('B1','B2',Relation.NOT_EQUAL)],
        condition_object_instance_names=[['B1'],['B1','B2']],
        condition_values=[True,True]
    ))
    #next_to_transivity
    state_constraints.append(StateConstraint(
        object_instances={'P1':place,'P2':place},
        head=next_to,
        head_value=True,
        head_object_instance_names=['P1','P2'],
        conditions=[next_to],
        condition_object_instance_names=[['P2','P1']],
        condition_values=[True]
    ))
    # fastened_transivity
    state_constraints.append(StateConstraint(
        object_instances={'B1':beam,'B2':beam, 'P1':pin},
        head=fastened,
        head_value=True,
        head_object_instance_names=['B1','B2', 'P1'],
        conditions=[fastened],
        condition_object_instance_names=[['B2','B1','P1']],
        condition_values=[True]
    ))
    #CWA_next_to
    state_constraints.append(StateConstraint(
        object_instances={'P1':place,'P2':place},
        head=next_to,
        head_object_instance_names=['P1','P2'],
        head_value=False,
        conditions=[next_to],
        condition_object_instance_names=[['P1','P2']],
        condition_values=[False]
    ))
    #objects can only occuy one location
    state_constraints.append(StateConstraint(
        object_instances={'P1':place,'P2':place, 'T':ob},
        head=location,
        head_object_instance_names=['T','P1'],
        head_value=False,
        conditions=[location, Property('P1','P2',Relation.NOT_EQUAL)],
        condition_object_instance_names=[['T','P2'],['P1','P2']],
        condition_values=[True,True]
    ))
    #if the robot is holding an object the object will move with it
    #this could alternatively be defined as a state constraint
    state_constraints.append(StateConstraint(
        object_instances={'T1':thing,'T2':thing,'P1':place, 'R':robot},
        head=location,
        head_object_instance_names=['T1','P1'],
        head_value=True,
        conditions=[location,in_hand,Property('T1','T2',Relation.EQUAL)],
        condition_object_instance_names=[['R','P1'],['R','T2'],['T1','T2']],
        condition_values=[True, True, True]
    ))

    #?beam domain state constraints
    capping_constraint = StateConstraint(
        object_instances={'B1':beam,'B2':beam,'B3':beam},
        head=is_capped_by,
        head_object_instance_names=['B1','B2','B3'],
        head_value=True,
        conditions=[fits_into,fits_into,Property('B2','B3',Relation.NOT_EQUAL)],
        condition_object_instance_names=[['B1','B2'],['B1','B3'],['B2','B3']],
        condition_values=[True,True,True]
    )
    capping_transivity = StateConstraint(
        object_instances={'B1':beam,'B2':beam,'B3':beam},
        head=is_capped_by,
        head_object_instance_names=['B1','B2','B3'],
        head_value=True,
        conditions=[is_capped_by],
        condition_object_instance_names=[['B1','B3','B2']],
        condition_values=[True]
    )
    ## beams supported if either end can be joined to another beam
    support_constraint = StateConstraint(
        object_instances={'B1':beam,'B2':beam},
        head=supported,
        head_object_instance_names=['B1'],
        head_value=True,
        conditions=[in_assembly,fits_into],
        condition_object_instance_names=[['B2'],['B1','B2']],
        condition_values=[True,True]
    )
    support_constraint2 = StateConstraint(
        object_instances={'B1':beam,'B2':beam},
        head=supported,
        head_object_instance_names=['B1'],
        head_value=True,
        conditions=[in_assembly,fits_into],
        condition_object_instance_names=[['B2'],['B2','B1']],
        condition_values=[True,True]
    )
    ##rules govenerning fit interactions
    inception = StateConstraint(
        object_instances={'B1':beam,'B2':beam},
        head=fits_into,
        head_object_instance_names=['B1','B2'],
        head_value=False,
        conditions=[fits_into],
        condition_object_instance_names=[['B2','B1']],
        condition_values=[True]
    )
    thru_ception = StateConstraint(
        object_instances={'B1':beam,'B2':beam},
        head=fits_through,
        head_object_instance_names=['B1','B2'],
        head_value=False,
        conditions=[fits_through],
        condition_object_instance_names=[['B2','B1']],
        condition_values=[True]
    )
    ##mutual exclusion
    thru_not_in = StateConstraint(
        object_instances={'B1':beam,'B2':beam},
        head=fits_into,
        head_object_instance_names=['B1','B2'],
        head_value=False,
        conditions=[fits_through],
        condition_object_instance_names=[['B1','B2']],
        condition_values=[True]
    )
    in_not_thru = StateConstraint(
        object_instances={'B1':beam,'B2':beam},
        head=fits_through,
        head_object_instance_names=['B1','B2'],
        head_value=False,
        conditions=[fits_into],
        condition_object_instance_names=[['B1','B2']],
        condition_values=[True]
    )
    ## self intersection
    not_in_self = StateConstraint(
        object_instances={'B1':beam,'B2':beam},
        head=fits_into,
        head_object_instance_names=['B1','B2'],
        head_value=False,
        conditions=[Property('B1','B2',Relation.EQUAL)],
        condition_object_instance_names=[['B1','B2']],
        condition_values=[True]
    )
    not_thru_self = StateConstraint(
        object_instances={'B1':beam,'B2':beam},
        head=fits_through,
        head_object_instance_names=['B1','B2'],
        head_value=False,
        conditions=[Property('B1','B2',Relation.EQUAL)],
        condition_object_instance_names=[['B1','B2']],
        condition_values=[True]
    )
    CWA_on_capped = StateConstraint(
        object_instances={'B1':beam,'B2':beam,'B3':beam},
        head=is_capped_by,
        head_value=False,
        head_object_instance_names=['B1','B2','B3'],
        conditions=[is_capped_by],
        condition_values=[False],
        condition_object_instance_names=[['B1','B2','B3']],
    )
                        
    state_constraints += [capping_constraint,
                        capping_transivity,
                        CWA_on_capped, 
                        support_constraint, 
                        support_constraint2,
                        not_in_self,
                        not_thru_self,
                        in_not_thru,
                        thru_not_in,
                        inception,
                        thru_ception]
    
    #CWA on fits_into_c
    state_constraints.append(StateConstraint(
        object_instances={'B1':beam,'B2':beam},
        head=fits_into,
        head_value=False,
        head_object_instance_names=['B1','B2'],
        conditions=[fits_into],
        condition_object_instance_names=[['B1','B2']],
        condition_values=[False]
    ))

    #!pick_up action
    pick_up = ActionDefinition('pick_up', [robot, thing])
    ##puts thing into robots hand
    pu_c1 = CausalLaw(
        action=pick_up,
        object_instances={'R':robot,'T':thing},
        action_object_instance_names=['R','T'],
        fluent_affected=in_hand,
        fluent_object_instance_names=['R','T'],
        fluent_value=True
    )
    ##cannot pick up unless in same location
    pu_ec2 = ExecutabilityCondition(
        action=pick_up,
        object_instances={'R':robot,'T1':thing, 'P1':place, 'P2':place},
        action_object_instance_names=['R','T1'],
        conditions=[location, location, Property('P1','P2',Relation.NOT_EQUAL)],
        condition_object_instance_names=[['T1','P1'],['R','P2'],['P1','P2']],
        condition_values=[True, True, True],
    )
    ##can only hold one item at once
    pu_ec3 = ExecutabilityCondition(
        action=pick_up,
        object_instances={'R':robot,'T1':thing, 'T2':thing},
        action_object_instance_names=['R','T1'],
        conditions=[in_hand, Property('thing','T2',Relation.IS_OF_SORT)],
        condition_object_instance_names=[['R','T2'],['T2']],
        condition_values=[True, True],
    )
    pick_up_action = Action(pick_up,[pu_c1],[pu_ec2, pu_ec3])

    #!move action
    move = ActionDefinition('move', [robot,place])
    m_c1 = CausalLaw(
        action=move,
        object_instances={'R':robot,'P':place},
        action_object_instance_names=['R','P'],
        fluent_object_instance_names=['R','P'],
        fluent_affected=location,
        fluent_value=True
        )
    # beam domain specific --> moving without releasing the pin will cause the joint to no longer be fastened
    m_c3 = CausalLaw(
        action=move,
        object_instances={'R':robot,'L':place,'P':pin,'B1':beam,'B2':beam},
        action_object_instance_names=['R','L'],
        fluent_object_instance_names=['B1','B2','P'],
        fluent_affected=fastened,
        fluent_value=False,
        conditions=[in_hand, fastened,
                    Property('beam','B1',Relation.IS_OF_SORT),
                    Property('beam','B2',Relation.IS_OF_SORT)],
        condition_values=[True, True,True,True],
        condition_object_instance_names=[['R','P'], ['B1','B2','P'],['B1'],['B2']],
        )
    ## cannot move to location you are already at
    m_ec1 = ExecutabilityCondition(
        action=move,
        object_instances={'R':robot,'P1':place,'P2':place},
        action_object_instance_names=['R','P1'],
        conditions=[location, Property('P1','P2',Relation.EQUAL)],
        condition_object_instance_names=[['R','P2'],['P1','P2']],
        condition_values=[True,True],
    )
    ## cannot move to location unless it is next to the current location
    m_ec2 = ExecutabilityCondition(
        action=move,
        object_instances={'R':robot,'P1':place,'P2':place},
        action_object_instance_names=['R','P1'],
        conditions=[location, next_to],
        condition_object_instance_names=[['R','P2'],['P1','P2']],
        condition_values=[True,False],
    )
    # cannot move after assembling or fastening a beam (need to release object)
    m_ec3 = ExecutabilityCondition(
        action=move,
        object_instances={'R':robot,'P1':place,'B1':beam,'P2':place},
        action_object_instance_names=['R','P1'],
        conditions=[in_hand,in_assembly],
        condition_object_instance_names=[['R','B1'],['B1']],
        condition_values=[True,True],
    )
    m_ec4 = ExecutabilityCondition(
        action=move,
        object_instances={'R':robot,'P1':place,'B1':beam,'B2':beam, 'P':pin},
        action_object_instance_names=['R','P1'],
        conditions=[in_hand,fastened],
        condition_object_instance_names=[['R','P'],['B1','B2','P']],
        condition_values=[True,True],
    )
    move_action = Action(move, [m_c1], [m_ec1,m_ec2, m_ec3, m_ec4])

    #!put_down action
    put_down = ActionDefinition('putdown',[robot, thing])
    ##cannot put down item which you are not holding
    pd_ec1 = ExecutabilityCondition(
        action = put_down,
        object_instances= {'R':robot, 'T':thing},
        action_object_instance_names=['R','T'],
        conditions= [in_hand],
        condition_object_instance_names= [['R','T']],
        condition_values= [False],
        )
    ##causes object to no longer be in robots hand
    pd_c1 = CausalLaw(put_down,
        in_hand,
        False,
        {'R':robot,'T':thing},
        ['R','T'],['R','T'],
        )
    put_down_action = Action(put_down,[pd_c1],[pd_ec1])


    #!assemble action
    assemble = ActionDefinition('assemble',[robot,beam])
    #causes in assembly
    asem_c1 = CausalLaw(
        action=assemble,
        fluent_affected=in_assembly,
        fluent_value=True,
        object_instances={'R':robot,'B':beam},
        action_object_instance_names=['R','B'],
        fluent_object_instance_names=['B'],
    )
    #assembly causes misaligned of other assembled beams (apart from base beam which is fixed)
    asem_c2 = CausalLaw(
        action = assemble,
        object_instances = {'R':robot, 'B1':beam, 'B2':beam},
        fluent_affected = misaligned,
        fluent_value=True,
        fluent_object_instance_names=['B2'],
        action_object_instance_names=['R','B1'],
        conditions=[in_assembly,base,Property('B1','B2',Relation.NOT_EQUAL)],
        condition_object_instance_names=[['B2'],['B2'],['B1','B2']],
        condition_values=[True, False, True],
    )
    #assembled beams can be fastened together
    asem_c3 = CausalLaw(
        action = assemble,
        object_instances = {'R':robot, 'B1':beam, 'B2':beam},
        fluent_affected = can_fasten,
        fluent_value=True,
        fluent_object_instance_names=['B1','B2'],
        action_object_instance_names=['R','B1'],
        conditions=[in_assembly,fits_into,Property('B1','B2',Relation.NOT_EQUAL)],
        condition_object_instance_names=[['B2'],['B1','B2'],['B1','B2']],
        condition_values=[True, True, True],
    )
    # need reverse for when we put a cap beam in place
    asem_c3a = CausalLaw(
        action = assemble,
        object_instances = {'R':robot, 'B1':beam, 'B2':beam},
        fluent_affected = can_fasten,
        fluent_value=True,
        fluent_object_instance_names=['B1','B2'],
        action_object_instance_names=['R','B2'],
        conditions=[in_assembly,fits_into,Property('B1','B2',Relation.NOT_EQUAL)],
        condition_object_instance_names=[['B1'],['B1','B2'],['B1','B2']],
        condition_values=[True, True, True],
    )
    asem_c4 = CausalLaw(
        action = assemble,
        object_instances = {'R':robot, 'B1':beam, 'B2':beam},
        fluent_affected = can_fasten,
        fluent_value=True,
        fluent_object_instance_names=['B1','B2'],
        action_object_instance_names=['R','B1'],
        conditions=[in_assembly,fits_through,Property('B1','B2',Relation.NOT_EQUAL)],
        condition_object_instance_names=[['B2'],['B1','B2'],['B1','B2']],
        condition_values=[True, True, True],
    )
    #beam must be held to assemble
    asem_ec1 = ExecutabilityCondition(
        action= assemble,
        object_instances={'R':robot,'B':beam},
        action_object_instance_names=['R','B'],
        conditions=[in_hand],
        condition_values=[False],
        condition_object_instance_names=[['R','B']],
    )
    # cannot add beam if both cap beams are already in place
    asem_ec2 = ExecutabilityCondition(
        action= assemble,
        object_instances={'R':robot,'B1':beam,'B2':beam,'B3':beam},
        action_object_instance_names=['R','B1'],
        conditions=[in_assembly,in_assembly,is_capped_by,
                    Property('B2','B3',Relation.NOT_EQUAL)],
        condition_values=[True,True,True,True],
        condition_object_instance_names=[['B2'],['B3'],['B1','B2','B3'],['B2','B3']],
    )
    #cannot add beam if another beam which needs to pass through the beam is alread in the assembly
    asem_ec3 = ExecutabilityCondition(
        action= assemble,
        object_instances={'R':robot,'B1':beam,'B2':beam},
        action_object_instance_names=['R','B1'],
        conditions=[in_assembly,fits_through],
        condition_values=[True,True],
        condition_object_instance_names=[['B2'],['B2','B1']],
    )
    #cannot add a beam which would cap another beam if the beams to be capped are not yet in the assembly
    asem_ec5 = ExecutabilityCondition(
        action= assemble,
        object_instances={'R':robot,'B1':beam,'B2':beam,'B3':beam},
        action_object_instance_names=['R','B1'],
        conditions=[in_assembly,in_assembly,is_capped_by,
                    Property('B2','B3',Relation.NOT_EQUAL)],
        condition_values=[False,True,True,True,True],
        condition_object_instance_names=[['B2'],['B3'],['B2','B1','B3'],['B2','B3']],
    )
    #cannot assemble unless in assembly area
    asem_ec6 = ExecutabilityCondition(
        action=assemble,
        object_instances={'R':robot,'P':place,'B':beam},
        action_object_instance_names=['R','B'],
        conditions=[location,
                    Property('P','assembly_area',Relation.NOT_EQUAL)],
        condition_object_instance_names=[['R','P'],['P']],
        condition_values=[True,True],
    )
    #cannot add beams already in the assembly
    asem_ec7 = ExecutabilityCondition(
        action=assemble,
        object_instances={'R':robot,'B':beam},
        action_object_instance_names=['R','B'],
        conditions=[in_assembly],
        condition_object_instance_names=[['B']],
        condition_values=[True],
    )
    #beams must be supported to add them
    asem_ec8 = ExecutabilityCondition(
        action=assemble,
        object_instances={'R':robot,'B':beam},
        action_object_instance_names=['R','B'],
        conditions=[supported],
        condition_object_instance_names=[['B']],
        condition_values=[False],
    )
    # cannot assemble if other beams are misaligned
    asem_ec9 = ExecutabilityCondition(
        action = assemble,
        object_instances={'R':robot, 'B1':beam, 'B2':beam},
        action_object_instance_names = ['R','B1'],
        conditions = [in_assembly,misaligned],
        condition_object_instance_names=[['B2'],['B2']],
        condition_values = [True,True],
    )
    #heuristic for assembling pins before other beams
    asem_ec10 = ExecutabilityCondition(
        action = assemble,
        object_instances={'R':robot, 'B1':beam, 'P':pin, 'B2':beam, 'B3':beam},
        action_object_instance_names = ['R','B1'],
        conditions = [can_fasten, Property('B1','B2',Relation.NOT_EQUAL), Property('B1','B3',Relation.NOT_EQUAL)],
        condition_object_instance_names=[['B2','B3'],['B1','B2'],['B1','B3']],
        condition_values = [True,True,True],
    )
    # assemble_action = Action(assemble,[asem_c1],[asem_ec1,asem_ec2,asem_ec3,asem_ec5,asem_ec6,asem_ec7,asem_ec8,asem_ec9])
    assemble_action = Action(assemble,
                             [asem_c1,asem_c2, asem_c3, asem_c3a, asem_c4],
                             [asem_ec1,asem_ec2,asem_ec3,asem_ec5,
                              asem_ec6,asem_ec7,asem_ec8,asem_ec9,
                              asem_ec10])

    #!Fasten Action
    fasten = ActionDefinition('fasten',[robot,beam,beam,pin])
    # causes fastened state
    f_c1 = CausalLaw(
        action=fasten,
        fluent_affected=fastened,
        fluent_value=True,
        object_instances={'R':robot,'B1':beam,'B2':beam,'P1':pin},
        action_object_instance_names=['R','B1','B2','P1'],
        fluent_object_instance_names=['B1','B2','P1'],
    )
    # causes can_fasten = False state
    f_c2 = CausalLaw(
        action=fasten,
        fluent_affected=can_fasten,
        fluent_value=False,
        object_instances={'R':robot,'B1':beam,'B2':beam,'P1':pin},
        action_object_instance_names=['R','B1','B2','P1'],
        fluent_object_instance_names=['B1','B2'],
    )
    ## parts must be assembled to be fastened
    f_ec1 = ExecutabilityCondition(
        action= fasten,
        object_instances={'R':robot,'B1':beam,'B2':beam,'P1':pin},
        action_object_instance_names=['R','B1','B2','P1'],
        conditions=[in_assembly],
        condition_values=[False],
        condition_object_instance_names=[['B1']],
    )
    ## both parts must be in assembly to fasten them
    f_ec2 = ExecutabilityCondition(
        action=fasten,
        object_instances={'R':robot,'B1':beam,'B2':beam,'P1':pin},
        action_object_instance_names=['R','B1','B2','P1'],
        conditions=[in_assembly],
        condition_values=[False],
        condition_object_instance_names=[['B2']],
    )
    ## must hold pin to do fastening
    f_ec3 = ExecutabilityCondition(
        action= fasten,
        object_instances={'R':robot,'B1':beam,'B2':beam,'P1':pin},
        action_object_instance_names=['R','B1','B2','P1'],
        conditions=[in_hand],
        condition_values=[False],
        condition_object_instance_names=[['R','P1']],
    )
    #can only fasten beams which fit together
    f_ec4 = ExecutabilityCondition(
        action = fasten,
        object_instances={'R':robot,'B1':beam,'B2':beam,'P1':pin},
        action_object_instance_names=['R','B1','B2','P1'],
        conditions=[fits_into],
        condition_values=[False],
        condition_object_instance_names=[['B1','B2']],
    )
    #can only fasten beams which dont already have a pin in
    f_ec5 = ExecutabilityCondition(
        action = fasten,
        object_instances={'R':robot,'B1':beam,'B2':beam,'P1':pin,'P2':pin},
        action_object_instance_names=['R','B1','B2','P1'],
        conditions=[fastened],
        condition_values=[True],
        condition_object_instance_names=[['B1','B2','P2']],
    )
    # must be at beam location to perform fastening
    f_ec6 = ExecutabilityCondition(
        action = fasten,
        object_instances={'R':robot,'B1':beam,'B2':beam,'P1':pin,'L1':place,'L2':place},
        action_object_instance_names=['R','B1','B2','P1'],
        conditions=[location,location,Property('L1','L2',Relation.NOT_EQUAL)],
        condition_values=[True,True,True],
        condition_object_instance_names=[['B1','L1'],['R','L2'],['L1','L2']]
    )
    # cannot fasten misaligned beams
    f_ec7 = ExecutabilityCondition(
        action = fasten,
        object_instances={'R':robot,'B1':beam,'B2':beam,'P1':pin},
        action_object_instance_names=['R','B1','B2','P1'],
        conditions=[misaligned],
        condition_values=[True],
        condition_object_instance_names=[['B1']]
    )
    f_ec8 = ExecutabilityCondition(
        action = fasten,
        object_instances={'R':robot,'B1':beam,'B2':beam,'P1':pin},
        action_object_instance_names=['R','B1','B2','P1'],
        conditions=[misaligned],
        condition_values=[True],
        condition_object_instance_names=[['B2']]
    )
    # cannot re-use pins
    f_ec9 = ExecutabilityCondition(
        action = fasten,
        object_instances={'R':robot,'B1':beam,'B2':beam,'B3':beam,'B4':beam,'P1':pin},
        action_object_instance_names=['R','B1','B2','P1'],
        conditions=[fastened],
        condition_values=[True],
        condition_object_instance_names=[['B3','B4','P1']]
    )
    fasten_action = Action(fasten,
                           [f_c1,f_c2],
                           [f_ec1,f_ec2,f_ec3,f_ec4,f_ec5,f_ec6, f_ec7, f_ec8, f_ec9])

    push = ActionDefinition('push',[robot,beam])
    # pushing causes beam to no longer be misaligned
    push_c1 = CausalLaw(
        action=push,
        fluent_affected=misaligned,
        fluent_value=False,
        object_instances={'R':robot,'B1':beam},
        action_object_instance_names=['R','B1'],
        fluent_object_instance_names=['B1'],
    )
    # must be at beam location to push
    push_ec1 = ExecutabilityCondition(
        action = push,
        object_instances={'R':robot,'B1':beam,'L1':place,'L2':place},
        action_object_instance_names=['R','B1'],
        conditions=[location,location,Property('L1','L2',Relation.NOT_EQUAL)],
        condition_values=[True,True,True],
        condition_object_instance_names=[['B1','L1'],['R','L2'],['L1','L2']]
    )
    # beam to push must be in assembly
    push_ec2 = ExecutabilityCondition(
        action = push,
        object_instances={'R':robot,'B1':beam},
        action_object_instance_names=['R','B1'],
        conditions=[in_assembly],
        condition_values=[False],
        condition_object_instance_names=[['B1']]
    )
    # cannot push whilst holding object
    push_ec3 = ExecutabilityCondition(
        action = push,
        object_instances={'R':robot,'B1':beam,'T':thing},
        action_object_instance_names=['R','B1'],
        conditions=[in_hand, Property('thing','T',Relation.IS_OF_SORT)],
        condition_values=[True,True],
        condition_object_instance_names=[['R','T'],['T']]
    )
    push_action = Action(push,[push_c1],[push_ec1,push_ec2, push_ec3])

    #!ALD
    # example with 5 beams
    ALD = ActionLangSysDesc(
            sorts=[robot,beam,pin,thing,ob,place],
            statics=statics,
            state_constraints=state_constraints,
            inertial_fluents=fluents,
            actions=[put_down_action,move_action,pick_up_action,assemble_action,fasten_action,push_action],
            domain_setup=[],
            goal_description=[],
            planning_steps=10)

    #ALD.complete_domain_setup_fluent(in_hand,False) #assert robot hand is empty
    #ALD.to_sparc_program().save(SAVE_DIR+'beam_domain_coarse.sp')
    return ALD
