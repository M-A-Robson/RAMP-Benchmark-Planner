from planner.sparc_planning.src.al_structures import *

SAVE_DIR = '/home/local/MTC_ORI_Collab/sparc_planning/sparc_files/'


def generate_fine_beam_domain():
    #!sorts
    robot = BasicSort('robot', ['rob0'])
    #add beam and pin sorts (simple 4 beam set)
    beam = BasicSort('beam',['b1','b2','b3','b4'])
    link = BasicSort('link',['l1','l2','l3','l4'])
    #!only end joints in this simple example set
    #TODO define the other joint types (mid_joints)
    in_m_end = BasicSort('in_m_end',['j3','j4','j7','j8'])
    in_f_end = BasicSort('in_f_end',['j1','j2','j5','j6'])
    angle_m_end = BasicSort('angle_m_end',[])
    thru_m = BasicSort('thru_m', [])
    angle_f = BasicSort('angle_f', [])
    thru_f = BasicSort('thru_f', [])
    joint = SuperSort('joint', [in_m_end,in_f_end,angle_m_end,thru_m,angle_f,thru_f])
    beam_part = SuperSort('beam_part',[link,joint]) #mid_joint
    pin = BasicSort('pin',['p1','p2','p3','p4'])
    thing = SuperSort('thing', [beam,pin])
    thing_part = SuperSort('thing_part', [beam_part, pin]) # graspable things
    ob = SuperSort('object', [robot, thing])
    place_c = BasicSort('place_c', ['input_area', 'intermediate_area', 'assembly_area'])
    non_placement_loc = BasicSort('non_placement_location',['above_input_area','above_intermediate_area','above_assembly_area'])
    # approach locations from xml extraction
    approach_loc = BasicSort('approach_location', ['b1a','b2a','b3a','b4a','p1a','p2a','p3a','p4a'])
    #through and pre-rotate locations from beam xml extraction
    #! no through or rotate beams in this set
    prerot_loc = BasicSort('prerot_location', []) 
    through_loc = BasicSort('through_location', [])
    # insert locatons for beams and pins from xml extraction
    target_loc = BasicSort('target_location', ['b1t','b2t','b3t','b4t','p1t','p2t','p3t','p4t'])
    # add a near_to location for all fine locations
    near_to_loc = BasicSort('near_to_location', ['nt_b1a','nt_b2a','nt_b3a','nt_b4a','nt_p1a','nt_p2a',
                                                'nt_p3a','nt_p4a','nt_b1t','nt_b2t','nt_b3t','nt_b4t',
                                                'nt_p1t','nt_p2t','nt_p3t','nt_p4t','nt_above_input',
                                                'nt_above_intermediate','nt_above_assembly'])
    input_locations = BasicSort('input_location', ['b2i','b3i','b4i','p1i','p2i','p3i','p4i'])
    assembly_loc = SuperSort('assembly_location', [approach_loc,target_loc,prerot_loc,through_loc])
    place_f = SuperSort('place_f', [assembly_loc, near_to_loc, input_locations, non_placement_loc])
    # need sets to allow for more than one refined sort type using the component keyword
    fine_res_sort = SuperSort('fine_res_sort',[place_f, thing_part])
    coarse_res_sort = SuperSort('coarse_res_sort',[place_c,thing])

    #!statics
    next_to_c = Func('next_to_c', [place_c,place_c], FuncType.STATIC)
    next_to_f = Func('next_to_f', [place_f,place_f], FuncType.STATIC)
    # add component relation
    component = Func('component',[coarse_res_sort,fine_res_sort], FuncType.STATIC)
    # define close locations for recovery/motion control
    near_to = Func('near_to', [near_to_loc,place_f], FuncType.STATIC)
    # add beam domain statics with refinements
    fits_into_c = Func('fits_into_c', [beam,beam], FuncType.STATIC)
    fits_into_f = Func('fits_into_f', [beam_part,beam_part], FuncType.STATIC)
    fits_through_c = Func('fits_through_c',[beam,beam],FuncType.STATIC)
    fits_through_f = Func('fits_through_f',[beam_part,beam_part],FuncType.STATIC)

    is_capped_by = Func('is_capped_by',[beam,beam,beam],FuncType.STATIC)
    # new fine res statics of beam domain
    ## beam part association
    connected_to = Func('connected_to',[beam_part,beam_part],FuncType.STATIC)
    between = Func('between',[beam_part,beam_part,beam_part],FuncType.STATIC)
    ## assembly location structure
    assem_target_loc = Func('assem_target_loc', [thing, target_loc],FuncType.STATIC)
    assem_approach_loc = Func('assem_approach_loc', [thing, approach_loc],FuncType.STATIC)
    #! no rotation or through beams in this set
    beam_prerotate_loc = Func('beam_prerotate_loc', [beam, prerot_loc],FuncType.STATIC)
    beam_through_loc = Func('beam_through_loc', [beam, through_loc],FuncType.STATIC)

    statics = [next_to_c,next_to_f,component,near_to,fits_into_c,
            fits_into_f,fits_through_c,fits_through_f,is_capped_by,
            connected_to,between, assem_approach_loc, assem_target_loc,
            beam_through_loc, beam_prerotate_loc]

    #!fluents
    in_hand_c = Func('in_hand_c', [robot,thing], FuncType.FLUENT)
    in_hand_f = Func('in_hand_f', [robot,thing_part], FuncType.FLUENT)
    coarse_location = Func('loc_c', [ob,place_c], FuncType.FLUENT)
    location = Func('loc_f', [ob,place_f], FuncType.FLUENT)
    #current_grasp_mode = Func('current_grasp_mode', [robot, grasp_mode], FuncType.FLUENT)
    #on = Func('on', [thing,thing], FuncType.FLUENT)
    #clear = Func('clear', [thing], FuncType.FLUENT)
    # add beam domain fluents
    in_assembly_c = Func('in_assembly_c',[beam],FuncType.FLUENT)
    in_assembly_f = Func('in_assembly_f',[beam_part],FuncType.FLUENT)
    supported_c = Func('supported_c',[beam], FuncType.FLUENT)
    supported_f = Func('supported_f',[beam_part], FuncType.FLUENT)
    fastened_c = Func('fastened_c',[beam,beam,pin], FuncType.FLUENT)
    fastened_f = Func('fastened_f',[beam_part,beam_part,pin],FuncType.FLUENT)
    fluents = [in_hand_c,in_hand_f,coarse_location,location,in_assembly_c,
            in_assembly_f,supported_c, supported_f,fastened_c,fastened_f]

    #!state constraints
    state_constraints=[]
    #only place_f can be component of place_c
    place_component_limit = StateConstraint(
        object_instances={'C1':place_f,'P1':place_c,},
        head=component,
        head_value=False,
        head_object_instance_names=['C1','P1'],
        conditions=[Property('place_c','P1',Relation.IS_OF_SORT),Property('place_f','C1',Relation.IS_OF_SORT)],
        condition_values=[True,False],
        condition_object_instance_names=[['P1'],['C1']]
    )
    state_constraints.append(place_component_limit)
    #only beam_part can be a component of beam
    state_constraints.append(StateConstraint(
        object_instances={'C1':beam_part,'P1':beam,},
        head=component,
        head_value=False,
        head_object_instance_names=['C1','P1'],
        conditions=[Property('beam','P1',Relation.IS_OF_SORT),Property('beam_part','C1',Relation.IS_OF_SORT)],
        condition_values=[True,False],
        condition_object_instance_names=[['P1'],['C1']]
    ))
    #next_to_transivity
    state_constraints.append(StateConstraint(
        object_instances={'C1':place_f,'C2':place_f},
        head=next_to_f,
        head_value=True,
        head_object_instance_names=['C1','C2'],
        conditions=[next_to_f],
        condition_object_instance_names=[['C2','C1']],
        condition_values=[True]
    ))
    #CWA_next_to
    state_constraints.append(StateConstraint(
        object_instances={'P1':place_c,'P2':place_c},
        head=next_to_c,
        head_object_instance_names=['P1','P2'],
        head_value=False,
        conditions=[next_to_c],
        condition_object_instance_names=[['P1','P2']],
        condition_values=[False]
    ))
    state_constraints.append(StateConstraint(
        object_instances={'P1':place_f,'P2':place_f},
        head=next_to_f,
        head_object_instance_names=['P1','P2'],
        head_value=False,
        conditions=[next_to_f],
        condition_object_instance_names=[['P1','P2']],
        condition_values=[False]
    ))
    #CWA near_to
    state_constraints.append(StateConstraint(
        object_instances={'P1':near_to_loc,'P2':place_f},
        head=near_to,
        head_object_instance_names=['P1','P2'],
        head_value=False,
        conditions=[near_to],
        condition_object_instance_names=[['P1','P2']],
        condition_values=[False]
    ))
    # near_to locations can be assigned parent locations automatically
    state_constraints.append(StateConstraint(
        object_instances={'C1':near_to_loc,'C2':place_f,'P1':place_c},
        head=component,
        head_value=True,
        head_object_instance_names=['P1','C1'],
        conditions=[near_to, component],
        condition_object_instance_names=[['C1','C2'], ['P1','C2']],
        condition_values=[True, True]
    ))
    #objects can only occuy one location
    state_constraints.append(StateConstraint(
        object_instances={'P1':place_f,'P2':place_f, 'T':thing},
        head=location,
        head_object_instance_names=['T','P1'],
        head_value=False,
        conditions=[location, Property('P1','P2',Relation.NOT_EQUAL)],
        condition_object_instance_names=[['T','P2'],['P1','P2']],
        condition_values=[True,True]
    ))
    #objects can only occuy one coarse location
    state_constraints.append(StateConstraint(
        object_instances={'P1':place_f,'P2':place_f, 'T':thing},
        head=coarse_location,
        head_object_instance_names=['T','P1'],
        head_value=False,
        conditions=[coarse_location, Property('P1','P2',Relation.NOT_EQUAL)],
        condition_object_instance_names=[['T','P2'],['P1','P2']],
        condition_values=[True,True]
    ))
    ##next_to bridge axiom
    next_to_f_c = StateConstraint(
        object_instances={'C1':place_f,'C2':place_f,'P1':place_c,'P2':place_c},
        head=next_to_c,
        head_value=True,
        head_object_instance_names=['P1','P2'],
        conditions=[next_to_f,component,component],
        condition_object_instance_names=[['C1','C2'],['P1','C1'],['P2','C2']],
        condition_values=[True,True,True]
        )
    state_constraints.append(next_to_f_c)
    #location bridge axiom
    loc_bridge_axiom = StateConstraint(
        object_instances={'C':place_f,'P':place_c,'T':thing},
        head=coarse_location,
        head_value=True,
        head_object_instance_names=['T','P'],
        conditions=[location,component],
        condition_object_instance_names=[['T','C'],['P','C']],
        condition_values=[True,True]
    )
    state_constraints.append(loc_bridge_axiom)
    #if the robot is holding an object the object will move with it
    state_constraints.append(StateConstraint(
        object_instances={'T1':thing,'T2':thing,'P1':place_f, 'R':robot},
        head=location,
        head_object_instance_names=['T1','P1'],
        head_value=True,
        conditions=[location,in_hand_c,Property('T1','T2',Relation.EQUAL)],
        condition_object_instance_names=[['R','P1'],['R','T2'],['T1','T2']],
        condition_values=[True, True, True]
    ))
    #only thing_parts can be components of things
    state_constraints.append(StateConstraint(
        object_instances={'C1':thing_part,'P1':thing,},
        head=component,
        head_value=False,
        head_object_instance_names=['C1','P1'],
        conditions=[Property('thing','P1',Relation.IS_OF_SORT),Property('thing_part','C1',Relation.IS_OF_SORT)],
        condition_values=[True,False],
        condition_object_instance_names=[['P1'],['C1']]
    ))
    #in_hand bridge axiom
    state_constraints.append(StateConstraint(
        object_instances={'T':thing,'R':robot,'TP':thing_part},
        head=in_hand_c,
        head_value=True,
        head_object_instance_names=['R','T'],
        conditions=[in_hand_f,component],
        condition_object_instance_names=[['R','TP'],['T','TP']],
        condition_values=[True,True]
    ))
    #supported bridge axioms
    state_constraints.append(StateConstraint(
        object_instances={'P1':beam_part,'B1':beam},
        head=supported_c,
        head_value=True,
        head_object_instance_names=['B1'],
        conditions=[supported_f,component],
        condition_object_instance_names=[['P1'],['B1','P1']],
        condition_values=[True,True]
        ))
    ##all parts of beam are supported if one is
    state_constraints.append(StateConstraint(
        object_instances={'P1':beam_part,'B1':beam},
        head=supported_f,
        head_value=True,
        head_object_instance_names=['P1'],
        conditions=[supported_c,component],
        condition_object_instance_names=[['B1'],['B1','P1']],
        condition_values=[True,True]
        ))
    #in_assembly bridge axioms
    state_constraints.append(StateConstraint(
        object_instances={'P1':beam_part,'B1':beam},
        head=in_assembly_c,
        head_value=True,
        head_object_instance_names=['B1'],
        conditions=[in_assembly_f,component],
        condition_object_instance_names=[['P1'],['B1','P1']],
        condition_values=[True,True]
        ))
    ##all parts of beam are in_assembly if one is
    state_constraints.append(StateConstraint(
        object_instances={'P1':beam_part,'B1':beam},
        head=in_assembly_f,
        head_value=True,
        head_object_instance_names=['P1'],
        conditions=[in_assembly_c,component],
        condition_object_instance_names=[['B1'],['B1','P1']],
        condition_values=[True,True]
        ))

    #?beam domain state constraints
    capping_constraint = StateConstraint(
        object_instances={'B1':beam,'B2':beam,'B3':beam},
        head=is_capped_by,
        head_object_instance_names=['B1','B2','B3'],
        head_value=True,
        conditions=[fits_into_c,fits_into_c,Property('B2','B3',Relation.NOT_EQUAL)],
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
    CWA_on_capped = StateConstraint(
        object_instances={'B1':beam,'B2':beam,'B3':beam},
        head=is_capped_by,
        head_value=False,
        head_object_instance_names=['B1','B2','B3'],
        conditions=[is_capped_by],
        condition_values=[False],
        condition_object_instance_names=[['B1','B2','B3']],
    )
    ## beams supported if either end can be joined to another beam
    support_constraint = StateConstraint(
        object_instances={'B1':beam,'B2':beam},
        head=supported_c,
        head_object_instance_names=['B1'],
        head_value=True,
        conditions=[in_assembly_c,fits_into_c],
        condition_object_instance_names=[['B2'],['B1','B2']],
        condition_values=[True,True]
    )
    support_constraint2 = StateConstraint(
        object_instances={'B1':beam,'B2':beam},
        head=supported_c,
        head_object_instance_names=['B1'],
        head_value=True,
        conditions=[in_assembly_c,fits_into_c],
        condition_object_instance_names=[['B2'],['B2','B1']],
        condition_values=[True,True]
    )
                        
    ## rules govenerning fit interactions
    ##TODO do we need these static rules if fits must be defined by designer anyway?
    inception = StateConstraint(
        object_instances={'B1':beam,'B2':beam},
        head=fits_into_c,
        head_object_instance_names=['B1','B2'],
        head_value=False,
        conditions=[fits_into_c],
        condition_object_instance_names=[['B2','B1']],
        condition_values=[True]
    )
    thru_ception = StateConstraint(
        object_instances={'B1':beam,'B2':beam},
        head=fits_through_c,
        head_object_instance_names=['B1','B2'],
        head_value=False,
        conditions=[fits_through_c],
        condition_object_instance_names=[['B2','B1']],
        condition_values=[True]
    )
    ##mutual exclusion
    thru_not_in = StateConstraint(
        object_instances={'B1':beam,'B2':beam},
        head=fits_into_c,
        head_object_instance_names=['B1','B2'],
        head_value=False,
        conditions=[fits_through_c],
        condition_object_instance_names=[['B1','B2']],
        condition_values=[True]
    )
    in_not_thru = StateConstraint(
        object_instances={'B1':beam,'B2':beam},
        head=fits_through_c,
        head_object_instance_names=['B1','B2'],
        head_value=False,
        conditions=[fits_into_c],
        condition_object_instance_names=[['B1','B2']],
        condition_values=[True]
    )
    ## self intersection
    not_in_self = StateConstraint(
        object_instances={'B1':beam,'B2':beam},
        head=fits_into_c,
        head_object_instance_names=['B1','B2'],
        head_value=False,
        conditions=[Property('B1','B2',Relation.EQUAL)],
        condition_object_instance_names=[['B1','B2']],
        condition_values=[True]
    )
    not_thru_self = StateConstraint(
        object_instances={'B1':beam,'B2':beam},
        head=fits_through_c,
        head_object_instance_names=['B1','B2'],
        head_value=False,
        conditions=[Property('B1','B2',Relation.EQUAL)],
        condition_object_instance_names=[['B1','B2']],
        condition_values=[True]
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

    #fastened bridge axiom
    state_constraints.append(StateConstraint(
        object_instances={'B1':beam,'B2':beam,'P':pin,'BP1':beam_part,'BP2':beam_part},
        head=fastened_c,
        head_value=True,
        head_object_instance_names=['B1','B2','P'],
        conditions=[fastened_f,component,component],
        condition_object_instance_names=[['BP1','BP2','P'],['B1','BP1'],['B2','BP2']],
        condition_values=[True,True,True]
    ))
    ##transivity of fastened
    state_constraints.append(
        StateConstraint(
            object_instances={'BP1':beam_part,'BP2':beam_part,'P':pin},
            head=fastened_f,
            head_value=True,
            head_object_instance_names=['BP1','BP2','P'],
            conditions=[fastened_f],
            condition_values=[True],
            condition_object_instance_names=[['BP2','BP1','P']],
        )
    )
    #?extended beam domain state constraints for new beam part connection relations
    ##transivity of beam part connections
    state_constraints.append(
        StateConstraint(
        object_instances={'BP1':beam_part,'BP2':beam_part},
        head=connected_to,
        head_value=True,
        head_object_instance_names=['BP1','BP2'],
        conditions=[connected_to],
        condition_values=[True],
        condition_object_instance_names=[['BP2','BP1']],)
    )

    ##CWA of beam part connections
    state_constraints.append(
        StateConstraint(
        object_instances={'BP1':beam_part,'BP2':beam_part},
        head=connected_to,
        head_value=False,
        head_object_instance_names=['BP1','BP2'],
        conditions=[connected_to],
        condition_values=[False],
        condition_object_instance_names=[['BP1','BP2']],)
    )

    ##between from connections
    state_constraints.append(
        StateConstraint(
        object_instances={'BP1':beam_part,'BP2':beam_part,'BP3':beam_part},
        head=between,
        head_value=True,
        head_object_instance_names=['BP1','BP2','BP3'],
        conditions=[connected_to,connected_to],
        condition_values=[True, True],
        condition_object_instance_names=[['BP1','BP2'],['BP1','BP3']],
        )
    )
    state_constraints.append(
        StateConstraint(
        object_instances={'BP1':beam_part,'BP2':beam_part,'BP3':beam_part,'BP4':beam_part},
        head=between,
        head_value=True,
        head_object_instance_names=['BP1','BP2','BP3'],
        conditions=[connected_to,between],
        condition_values=[True, True],
        condition_object_instance_names=[['BP1','BP2'],['BP4','BP1','BP3']],
        )
    )
    state_constraints.append(
        StateConstraint(
        object_instances={'BP1':beam_part,'BP2':beam_part,
                        'BP3':beam_part,'BP4':beam_part,
                        'BP5':beam_part},
        head=between,
        head_value=True,
        head_object_instance_names=['BP1','BP2','BP3'],
        conditions=[between,between],
        condition_values=[True, True],
        condition_object_instance_names=[['BP4','BP1','BP3'],['BP5','BP1','BP3']],
        )
    )

    #transivity of between
    state_constraints.append(
        StateConstraint(
        object_instances={'BP1':beam_part,'BP2':beam_part,
                        'BP3':beam_part},
        head=between,
        head_value=True,
        head_object_instance_names=['BP1','BP2','BP3'],
        conditions=[between],
        condition_values=[True],
        condition_object_instance_names=[['BP1','BP3','BP2']],
        )
    )
    # beam part fits_in and fits_through bridge axioms
    state_constraints.append(StateConstraint(
        object_instances={'B1':beam,'D1':beam_part,'B2':beam,'D2':beam_part},
        head=fits_into_c,
        head_value=True,
        head_object_instance_names=['B1','B2'],
        conditions=[fits_into_f,component, component],
        condition_object_instance_names=[['D1','D2'],['B1','D1'],['B2','D2']],
        condition_values=[True,True,True]
    ))
    state_constraints.append(StateConstraint(
        object_instances={'B1':beam,'D1':beam_part,'B2':beam,'D2':beam_part},
        head=fits_through_c,
        head_value=True,
        head_object_instance_names=['B1','B2'],
        conditions=[fits_through_f,component, component],
        condition_object_instance_names=[['D1','D2'],['B1','D1'],['B2','D2']],
        condition_values=[True,True,True]
    ))

    #! action refinements and new actions at fine resolution
    #removed on and clear as we can assume these for beam domain (no object stacking)
    #!pick_up_f action
    pick_up = ActionDefinition('pick_up_f', [robot, thing_part])
    ##puts thing part into robots hand
    pu_c1 = CausalLaw(
        action=pick_up,
        object_instances={'R':robot,'T':thing_part},
        action_object_instance_names=['R','T'],
        fluent_affected=in_hand_f,
        fluent_object_instance_names=['R','T'],
        fluent_value=True
    )
    ##cannot pick up unless in same location (for things without components i.e. pins are both things and thing_parts)
    pu_ec1 = ExecutabilityCondition(
        action=pick_up,
        object_instances={'R':robot,'P':thing_part,'P1':place_f, 'P2':place_f},
        action_object_instance_names=['R','P'],
        conditions=[location, location, Property('P1','P2',Relation.NOT_EQUAL)],
        condition_object_instance_names=[['P','P1'],['R','P2'],['P1','P2']],
        condition_values=[True, True, True],
    )
    ##cannot pick up unless in same location
    pu_ec2 = ExecutabilityCondition(
        action=pick_up,
        object_instances={'R':robot,'T1':thing,'TP':thing_part,'P1':place_f, 'P2':place_f},
        action_object_instance_names=['R','TP'],
        conditions=[location, location, Property('P1','P2',Relation.NOT_EQUAL), component],
        condition_object_instance_names=[['T1','P1'],['R','P2'],['P1','P2'],['T1','TP']],
        condition_values=[True, True, True, True],
    )
    ##can only hold one item at once
    pu_ec3 = ExecutabilityCondition(
        action=pick_up,
        object_instances={'R':robot,'T1':thing_part, 'T2':thing_part},
        action_object_instance_names=['R','T1'],
        conditions=[in_hand_f, Property('thing_part','T2',Relation.IS_OF_SORT)],
        condition_object_instance_names=[['rob0','T2'],['T2']],
        condition_values=[True, True],
    )
    ## can only pick up beams by the link elements
    pu_ec4 = ExecutabilityCondition(
        action=pick_up,
        object_instances={'R':robot,'T1':thing_part, 'B':beam},
        action_object_instance_names=['R','T1'],
        conditions=[component, Property('link','T1',Relation.IS_OF_SORT),Property('beam','B',Relation.IS_OF_SORT)],
        condition_object_instance_names=[['B','T1'],['T1'],['B']],
        condition_values=[True, False, True],
    )
    pick_up_action = Action(pick_up,[pu_c1,],[pu_ec1, pu_ec2, pu_ec3, pu_ec4])

    #!move_f action
    move = ActionDefinition('move_f', [robot,place_f])
    ## move changes robot location
    m_c1 = CausalLaw(
        action=move,
        object_instances={'R':robot,'P':place_f},
        action_object_instance_names=['R','P'],
        fluent_object_instance_names=['R','P'],
        fluent_affected=location,
        fluent_value=True
        )
    ## cannot move to location you are already at
    m_ec1 = ExecutabilityCondition(
        action=move,
        object_instances={'R':robot,'P1':place_f,'P2':place_f},
        action_object_instance_names=['R','P1'],
        conditions=[location, Property('P1','P2',Relation.EQUAL)],
        condition_object_instance_names=[['R','P2'],['P1','P2']],
        condition_values=[True,True],
    )
    ## cannot move to location unless it is next to the current location
    m_ec2 = ExecutabilityCondition(
        action=move,
        object_instances={'R':robot,'P1':place_f,'P2':place_f},
        action_object_instance_names=['R','P1'],
        conditions=[location, next_to_f, Property('P1','P2', Relation.NOT_EQUAL)],
        condition_object_instance_names=[['R','P2'],['P1','P2'],['P1','P2']],
        condition_values=[True,False,True],
    )
    # cannot move after assembling a beam (need to release object first)
    m_ec3 = ExecutabilityCondition(
        action=move,
        object_instances={'R':robot,'P1':place_f,'BP':beam_part,'P2':place_f},
        action_object_instance_names=['R','P1'],
        conditions=[in_hand_f,in_assembly_f],
        condition_object_instance_names=[['R','BP'],['BP']],
        condition_values=[True,True],
    )
    m_ec4 = ExecutabilityCondition(
        action=move,
        object_instances={'R':robot,'P1':place_f,'B1':beam,'B2':beam, 'P':pin},
        action_object_instance_names=['R','P1'],
        conditions=[in_hand_f,fastened_c],
        condition_object_instance_names=[['R','P'],['B1','B2','P']],
        condition_values=[True,True],
    )
    #cannot use move action from near_to locaitons (force use of move_local)
    m_ec5 = ExecutabilityCondition(
        action=move,
        object_instances={'R':robot,'P1':place_f,'P2':place_f},
        action_object_instance_names=['R','P2'],
        conditions=[location,near_to],
        condition_values=[True,True],
        condition_object_instance_names=[['R','P1'],['P1','P2']]
    )
    #cannot move to target locations whilst holding objects
    m_ec6 = ExecutabilityCondition(
        action=move,
        object_instances={'R':robot,'P1':place_f,'T1':thing},
        action_object_instance_names=['R','P1'],
        conditions=[in_hand_c,Property('target_location','P1',Relation.IS_OF_SORT)],
        condition_object_instance_names=[['R','T1'],['P1']],
        condition_values=[True,True],
    )
    #cannot move to pre-rotate locations whilst holding objects
    m_ec7 = ExecutabilityCondition(
        action=move,
        object_instances={'R':robot,'P1':place_f,'T1':thing},
        action_object_instance_names=['R','P1'],
        conditions=[in_hand_c,Property('through_location','P1',Relation.IS_OF_SORT)],
        condition_object_instance_names=[['R','T1'],['P1']],
        condition_values=[True,True],
    )
    #cannot move to through locations whilst holding objects
    m_ec8 = ExecutabilityCondition(
        action=move,
        object_instances={'R':robot,'P1':place_f,'T1':thing},
        action_object_instance_names=['R','P1'],
        conditions=[in_hand_c,Property('prerot_location','P1',Relation.IS_OF_SORT)],
        condition_object_instance_names=[['R','T1'],['P1']],
        condition_values=[True,True],
    )
    move_action = Action(move, [m_c1], [m_ec1,m_ec2, m_ec3, m_ec4, m_ec5, m_ec6, m_ec7, m_ec8])

    #!put_down_f action
    put_down = ActionDefinition('putdown_f',[robot, thing_part])
    ##cannot put down item which you are not holding
    pd_ec1 = ExecutabilityCondition(
        action = put_down,
        object_instances= {'R':robot, 'T':thing_part},
        action_object_instance_names=['R','T'],
        conditions= [in_hand_f],
        condition_object_instance_names= [['R','T']],
        condition_values= [False],
        )
    # cannot place objects at #non_placement_location's
    pd_ec2 = ExecutabilityCondition(
        action=put_down,
        object_instances={'R':robot, 'T':thing_part, 'C':place_f},
        action_object_instance_names=['R','T'],
        conditions=[location ,Property('non_placement_location','C',Relation.IS_OF_SORT)],
        condition_object_instance_names=[['R','C'],['C']],
        condition_values=[True,True]
    )
    ##causes object part to no longer be in robots hand
    pd_c1 = CausalLaw(put_down,
        in_hand_f,
        False,
        {'R':robot,'T':thing_part},
        ['R','T'],['R','T'],
        )
    # causes object to no longer be in hand
    pd_c2 = CausalLaw(
        action=put_down,
        fluent_affected=in_hand_c,
        fluent_value=False,
        object_instances={'R':robot,'TP':thing_part, 'T':thing},
        action_object_instance_names=['R','TP'],
        fluent_object_instance_names=['R','T'],
        conditions=[component],
        condition_object_instance_names=[['T','TP']],
        condition_values=[True],
        )
    put_down_action = Action(put_down,[pd_c1, pd_c2],[pd_ec1, pd_ec2])

    #?beam assembly action set
    #!assem_f_square
    assemble_square = ActionDefinition('assemble_f_square',[robot,beam_part])
    # puts beam into position as long as the part inserted is not an angle end (which needs further rotating)
    asem_sq_c1 = CausalLaw(
        action=assemble_square,
        fluent_affected=location,
        fluent_value=True,
        object_instances={'R':robot,'B':beam,'BP':beam_part,'C1':place_f},
        action_object_instance_names=['R','BP'],
        fluent_object_instance_names=['R','C'],
        conditions=[assem_target_loc,component, Property('angle_m_end', 'BP', Relation.IS_OF_SORT)],
        condition_object_instance_names=[['B','C'],['B','BP'],['BP']],
        condition_values=[True,True,False],
    )
    #puts angle beams at pre-rotate position
    asem_sq_c2 = CausalLaw(
        action=assemble_square,
        fluent_affected=location,
        fluent_value=True,
        object_instances={'R':robot,'B':beam,'BP':beam_part,'C1':place_f},
        action_object_instance_names=['R','BP'],
        fluent_object_instance_names=['R','C'],
        conditions=[beam_prerotate_loc,component, Property('angle_m_end', 'BP', Relation.IS_OF_SORT)],
        condition_object_instance_names=[['B','C'],['B','BP'],['BP']],
        condition_values=[True,True,True],
    )
    # puts beam_part into assembly (unless angle end type)
    asem_sq_c3 = CausalLaw(
        action=assemble_square,
        fluent_affected=in_assembly_f,
        fluent_value=True,
        object_instances={'R':robot,'BP':beam_part},
        action_object_instance_names=['R','BP'],
        fluent_object_instance_names=['BP'],
        conditions=[Property('angle_m_end', 'BP', Relation.IS_OF_SORT)],
        condition_object_instance_names=[['BP']],
        condition_values=[False],
    )
    # cannot be used to insert capping beam
    asem_sq_e1 = ExecutabilityCondition(
        action = assemble_square,
        object_instances={'R':robot,'B1':beam,'B2':beam,'B3':beam,'BP':beam_part},
        action_object_instance_names=['R','BP'],
        conditions=[is_capped_by,component],
        condition_values=[True,True],
        condition_object_instance_names=[['B1','B2','B3'],['B2','BP']],
    )
    # must be at approach location (or through_location)
    asem_sq_e2 = ExecutabilityCondition(
        action = assemble_square,
        object_instances={'R':robot,'B1':beam,'C':place_f,'BP':beam_part},
        action_object_instance_names=['R','BP'],
        conditions=[component,location,assem_approach_loc,Property('C1','C2',Relation.NOT_EQUAL)],
        condition_values=[True,True,True,True],
        condition_object_instance_names=[['B1','BP'],['R','C1'],['B1','C2'],['C1','C2']],
    )
    # cannot assemble unless the part into which this part fits is already in the assembly
    asem_sq_e3 = ExecutabilityCondition(
        action = assemble_square,
        object_instances={'R':robot,'BP1':beam_part, 'BP2':beam_part},
        action_object_instance_names=['R','BP1'],
        conditions=[fits_into_f, in_assembly_f],
        condition_values=[True, False],
        condition_object_instance_names=[['BP1','BP2'],['BP2']],
    )
    assemble_square_action = Action(assemble_square,[asem_sq_c1,asem_sq_c2, asem_sq_c3],[asem_sq_e1, asem_sq_e2, asem_sq_e3])

    #! assem_f_cap
    assemble_cap = ActionDefinition('assemble_f_cap',[robot,beam_part])
    # puts beam into position
    asem_cap_c1 = CausalLaw(
        action=assemble_cap,
        fluent_affected=location,
        fluent_value=True,
        object_instances={'R':robot,'B':beam,'BP':beam_part,'C1':place_f},
        action_object_instance_names=['R','BP'],
        fluent_object_instance_names=['R','C'],
        conditions=[assem_target_loc,component],
        condition_object_instance_names=[['B','C'],['B','BP']],
        condition_values=[True,True],
    )
    # adds beam to assembly
    asem_cap_c2 = CausalLaw(
        action=assemble_cap,
        fluent_affected=in_assembly_f,
        fluent_value=True,
        object_instances={'R':robot,'BP':beam_part},
        action_object_instance_names=['R','BP'],
        fluent_object_instance_names=['BP'],
    )

    # cannot be used for beams which get capped, unless it needs to cap other beams
    asem_cap_e1 = ExecutabilityCondition(
        action = assemble_cap,
        object_instances={'R':robot,'B1':beam,'B2':beam,'B3':beam,'B3':beam,'B3':beam,'BP':beam_part},
        action_object_instance_names=['R','BP'],
        conditions=[is_capped_by,component,is_capped_by],
        condition_values=[True,True,False],
        condition_object_instance_names=[['B1','B2','B3'],['B1','BP'],['B4','B1','B5']],
    )
    # must be at approach location
    asem_cap_e2 = ExecutabilityCondition(
        action = assemble_cap,
        object_instances={'R':robot,'B1':beam,'C':place_f,'BP':beam_part},
        action_object_instance_names=['R','BP'],
        conditions=[component,location,assem_approach_loc,Property('C1','C2',Relation.NOT_EQUAL)],
        condition_values=[True,True,True,True],
        condition_object_instance_names=[['B1','BP'],['R','C1'],['B1','C2'],['C1','C2']],
    )
    assemble_cap_action = Action(assemble_cap,[asem_cap_c1,asem_cap_c2],[asem_cap_e1, asem_cap_e2])

    #! assem_f_rotate
    assemble_rot = ActionDefinition('assemble_f_rotate',[robot,beam_part])
    # puts beam into target position
    asem_rot_c1 = CausalLaw(
        action=assemble_rot,
        fluent_affected=location,
        fluent_value=True,
        object_instances={'R':robot,'B':beam,'BP':beam_part,'C1':place_f},
        action_object_instance_names=['R','BP'],
        fluent_object_instance_names=['R','C'],
        conditions=[assem_target_loc,component],
        condition_object_instance_names=[['B','C'],['B','BP']],
        condition_values=[True,True],
    )
    # adds beam to assembly
    asem_rot_c2 = CausalLaw(
        action=assemble_rot,
        fluent_affected=in_assembly_f,
        fluent_value=True,
        object_instances={'R':robot,'BP':beam_part},
        action_object_instance_names=['R','BP'],
        fluent_object_instance_names=['BP'],
    )
    # cannot assemble unless the part into which this part fits is already in the assembly
    asem_rot_e1 = ExecutabilityCondition(
        action = assemble_rot,
        object_instances={'R':robot,'BP1':beam_part, 'BP2':beam_part},
        action_object_instance_names=['R','BP1'],
        conditions=[fits_into_f, in_assembly_f],
        condition_values=[True, False],
        condition_object_instance_names=[['BP1','BP2'],['BP2']],
    )
    # must be at pre-rotate location
    asem_rot_e2 = ExecutabilityCondition(
        action = assemble_rot,
        object_instances={'R':robot,'B1':beam,'C':place_f,'BP':beam_part},
        action_object_instance_names=['R','BP'],
        conditions=[component,location,beam_prerotate_loc,Property('C1','C2',Relation.NOT_EQUAL)],
        condition_values=[True,True,True,True],
        condition_object_instance_names=[['B1','BP'],['R','C1'],['B1','C2'],['C1','C2']],
    )
    assemble_rot_action = Action(assemble_rot,[asem_rot_c1,asem_rot_c2],[asem_rot_e1, asem_rot_e2])

    #! assem_f_through
    assemble_through = ActionDefinition('assemble_f_through',[robot,beam_part])
    # puts beam into 'through' position
    asem_th_c1 = CausalLaw(
        action=assemble_through,
        fluent_affected=location,
        fluent_value=True,
        object_instances={'R':robot,'B':beam,'BP':beam_part,'C1':place_f},
        action_object_instance_names=['R','BP'],
        fluent_object_instance_names=['R','C'],
        conditions=[beam_through_loc,component],
        condition_object_instance_names=[['B','C'],['B','BP']],
        condition_values=[True,True],
    )
    # must be at approach location
    asem_th_e1 = ExecutabilityCondition(
        action = assemble_through,
        object_instances={'R':robot,'B1':beam,'C':place_f,'BP':beam_part},
        action_object_instance_names=['R','BP'],
        conditions=[component,location,assem_approach_loc,Property('C1','C2',Relation.NOT_EQUAL)],
        condition_values=[True,True,True,True],
        condition_object_instance_names=[['B1','BP'],['R','C1'],['B1','C2'],['C1','C2']],
    )
    assemble_through_action = Action(assemble_through,[asem_th_c1],[asem_th_e1])


    # collect new assembly actions
    assembly_actions = [assemble_square_action,
                        assemble_cap_action,]
                        #assemble_through_action,
                        #assemble_rot_action] 

    #! these rules hold for all assemble_f actions (refinements of high level assembly action conditions)
    for ac in assembly_actions:
        # assembly actions cause already assembled parts to move to locations near_to their target_locations (low prob)
        # base beam doesnt get assigned a near to location so that it stays still
        ac.causal_laws.append(
            CausalLaw(
            action=ac.action_def,
            fluent_affected=location,
            fluent_value=True,
            object_instances={'R':robot,'B1':beam,'B2':beam,'BP':beam_part,'C1':place_f,'C2':place_f},
            action_object_instance_names=['R','BP'],
            fluent_object_instance_names=['B1','C1'],
            conditions=[in_assembly_c,near_to,location],
            condition_object_instance_names=[['B1'],['C1','C2'],['B1','C2']],
            condition_values=[True,True,True],
            )
        )
        # beam must be held to assemble
        ac.executability_conditions.append(
            ExecutabilityCondition(
                action = ac.action_def,
                object_instances={'R':robot,'B':beam,'BP':beam_part},
                action_object_instance_names=['R','BP'],
                conditions=[in_hand_c, component],
                condition_values=[False, True],
                condition_object_instance_names=[['R','B'], ['B','BP']],
            )
        )
        # cannot insert the part you are holding
        ac.executability_conditions.append(
            ExecutabilityCondition(
                action = ac.action_def,
                object_instances={'R':robot,'BP':beam_part},
                action_object_instance_names=['R','BP'],
                conditions=[in_hand_f],
                condition_object_instance_names=[['R','BP']],
                condition_values=[True]
            )
        )
        # cannot add beam if both cap beams are already in place
        ac.executability_conditions.append(
            ExecutabilityCondition(
                action = ac.action_def,
                object_instances={'R':robot,'BP':beam_part,'B1':beam,'B2':beam,'B3':beam},
                action_object_instance_names=['R','BP'],
                conditions=[in_assembly_c,in_assembly_c,is_capped_by,
                            Property('B2','B3',Relation.NOT_EQUAL),component],
                condition_values=[True,True,True,True,True],
                condition_object_instance_names=[['B2'],['B3'],['B1','B2','B3'],['B2','B3'],['B1','BP']],
            )
        )
        #cannot add beam if another beam which needs to pass through the beam is already in the assembly
        ac.executability_conditions.append(
            ExecutabilityCondition(
                action = ac.action_def,
                object_instances={'R':robot,'BP':beam_part,'BP2':beam_part},
                action_object_instance_names=['R','BP'],
                conditions=[in_assembly_f,fits_through_f],
                condition_values=[True,True],
                condition_object_instance_names=[['BP2'],['BP2','BP']],
            )
        )
        #cannot add a beam which would cap another beam if the beams to be capped are not yet in the assembly
        ac.executability_conditions.append(
            ExecutabilityCondition(
                action = ac.action_def,
                object_instances={'R':robot,'B1':beam,'B2':beam,'B3':beam,'BP':beam_part},
                action_object_instance_names=['R','BP'],
                conditions=[in_assembly_c,in_assembly_c,is_capped_by,
                            Property('B2','B3',Relation.NOT_EQUAL), component],
                condition_values=[False,True,True,True,True],
                condition_object_instance_names=[['B2'],['B3'],['B2','B1','B3'],['B2','B3'],['B1','BP']],
            )
        )
        #cannot add beams already in the assembly
        ac.executability_conditions.append(
            ExecutabilityCondition(
                action=ac.action_def,
                object_instances={'R':robot,'BP':beam_part},
                action_object_instance_names=['R','BP'],
                conditions=[in_assembly_f],
                condition_object_instance_names=[['BP']],
                condition_values=[True],
            )
        )
        #beams must be supported to add them
        ac.executability_conditions.append(
            ExecutabilityCondition(
                action=ac.action_def,
                object_instances={'R':robot,'BP':beam_part},
                action_object_instance_names=['R','BP'],
                conditions=[supported_f],
                condition_object_instance_names=[['BP']],
                condition_values=[False],
            )   
        )
        # add new condition forcing all other beams in connection to be at target location's before new beam insertion
        ac.executability_conditions.append(
            ExecutabilityCondition(
                action = ac.action_def,
                object_instances={'R':robot,'B1':beam,'B2':beam,'BP':beam_part},
                action_object_instance_names=['R','BP'],
                conditions=[in_assembly_c,Property('beam','B2',Relation.IS_OF_SORT),
                            location,assem_target_loc,
                            Property('C1','C2',Relation.NOT_EQUAL)],
                condition_object_instance_names=[['B2'],['B2'],['B2','C1'],['B2','C2'],['C1','C2']],
                condition_values=[True,True,True,True,True],
            )   
        )
        # assembly actions are only possible from assembly locations
        ac.executability_conditions.append(
            ExecutabilityCondition(
                action = ac.action_def,
                object_instances={'R':robot,'P':place_f,'BP':beam_part},
                action_object_instance_names=['R','BP'],
                conditions=[location, Property('assembly_location','P',Relation.IS_OF_SORT)],
                condition_values=[True,False],
                condition_object_instance_names=[['R','P'], ['P']],
            )
        )        

    # refined fasten action to use beam_parts
    #!Fasten Action 
    fasten = ActionDefinition('fasten',[robot,joint,joint,pin])
    ## causes beam parts to be fastened by pin
    f_c1 = CausalLaw(
        action=fasten,
        fluent_affected=fastened_f,
        fluent_value=True,
        object_instances={'R':robot,'B1':joint,'B2':joint,'P1':pin},
        action_object_instance_names=['R','B1','B2','P1'],
        fluent_object_instance_names=['B1','B2','P1']
    )
    ## causes movment to target_location for pin
    f_c2 = CausalLaw(
        action=fasten,
        fluent_affected=location,
        fluent_value=True,
        object_instances={'R':robot,'B1':joint,'B2':joint,'P1':pin,'C':place_f},
        action_object_instance_names=['R','B1','B2','P1'],
        fluent_object_instance_names=['R','C1'],
        conditions=[assem_target_loc],
        condition_object_instance_names=[['P1','C1']],
        condition_values=[True]
    )
    ## both beams must be in assembly to fasten them
    f_ec1 = ExecutabilityCondition(
        action= fasten,
        object_instances={'R':robot,'BP1':joint,'BP2':joint,'P1':pin},
        action_object_instance_names=['R','BP1','BP2','P1'],
        conditions=[in_assembly_f],
        condition_values=[False],
        condition_object_instance_names=[['BP1']]
    )
    f_ec2 = ExecutabilityCondition(
        action= fasten,
        object_instances={'R':robot,'BP1':joint,'BP2':joint,'P1':pin},
        action_object_instance_names=['R','BP1','BP2','P1'],
        conditions=[in_assembly_f],
        condition_values=[False],
        condition_object_instance_names=[['BP2']]
    )
    ## must hold pin to do fastening
    f_ec3 = ExecutabilityCondition(
        action= fasten,
        object_instances={'R':robot,'B1':joint,'B2':joint,'P1':pin},
        action_object_instance_names=['R','B1','B2','P1'],
        conditions=[in_hand_f],
        condition_values=[False],
        condition_object_instance_names=[['R','P1']],
    )
    #parts must be at target_locations
    f_ec4 = ExecutabilityCondition(
        action= fasten,
        object_instances={'R':robot,'B1':beam,'BP1':joint,'BP2':joint,'P1':pin,'C1':place_f,'C2':place_f},
        action_object_instance_names=['R','BP1','BP2','P1'],
        conditions=[location,assem_target_loc,component,Property('C1','C2',Relation.NOT_EQUAL)],
        condition_values=[True,True,True,True],
        condition_object_instance_names=[['B1','C1'],['B1','C2'],['B1','BP1'],['C1','C2']],
    )
    f_ec5 = ExecutabilityCondition(
        action= fasten,
        object_instances={'R':robot,'B1':beam,'BP1':joint,'BP2':joint,'P1':pin,'C1':place_f,'C2':place_f},
        action_object_instance_names=['R','BP1','BP2','P1'],
        conditions=[location,assem_target_loc,component,Property('C1','C2',Relation.NOT_EQUAL)],
        condition_values=[True,True,True,True],
        condition_object_instance_names=[['B1','C1'],['B1','C2'],['B1','BP2'],['C1','C2']],
    )
    #can only be used at pin approach location
    f_ec6 = ExecutabilityCondition(
        action= fasten,
        object_instances={'R':robot,'BP1':joint,'BP2':joint,'P1':pin,'C1':place_f,'C2':place_f},
        action_object_instance_names=['R','BP1','BP2','P1'],
        conditions=[location,assem_approach_loc,Property('C1','C2',Relation.NOT_EQUAL)],
        condition_values=[True,True,True],
        condition_object_instance_names=[['R','C1'],['P1','C2'],['C1','C2']],
    )
    #can only fasten joints which fit together
    f_ec7 = ExecutabilityCondition(
        action = fasten,
        object_instances={'R':robot,'BP1':joint,'BP2':joint,'P1':pin},
        action_object_instance_names=['R','BP1','BP2','P1'],
        conditions=[fits_into_f],
        condition_values=[False],
        condition_object_instance_names=[['BP1','BP2']],
    )
    #can only fasten joints which dont already have a pin in
    f_ec8 = ExecutabilityCondition(
        action = fasten,
        object_instances={'R':robot,'BP1':joint,'BP2':joint,'P1':pin,'P2':pin},
        action_object_instance_names=['R','BP1','BP2','P1'],
        conditions=[fastened_f],
        condition_values=[True],
        condition_object_instance_names=[['BP1','BP2','P2']],
    )
    fasten_action = Action(fasten,[f_c1,f_c2],
                        [f_ec1,f_ec2,f_ec3,f_ec4,f_ec5,f_ec6,f_ec7,f_ec8])


    #? new fine res actions
    #!push action for beam adjustment when in assembly
    push = ActionDefinition('push',[robot,beam])
    ##push causes beams to move to their target locations
    push_c1 = CausalLaw(
        action=push,
        object_instances={'R':robot,'B':beam,'C':place_f},
        action_object_instance_names=['R','B'],
        fluent_affected=location,
        fluent_object_instance_names=['B','C'],
        fluent_value=True,
        conditions=[assem_target_loc],
        condition_object_instance_names=[['B','C']],
        condition_values=[True]
    )
    ##push also causes robot to move to the beam target location
    push_c2 = CausalLaw(
        action=push,
        object_instances={'R':robot,'B':beam,'C':place_f},
        action_object_instance_names=['R','B'],
        fluent_affected=location,
        fluent_object_instance_names=['R','C'],
        fluent_value=True,
        conditions=[assem_target_loc],
        condition_object_instance_names=[['B','C']],
        condition_values=[True]
    )
    #beam must already be in assembly
    push_ec1 = ExecutabilityCondition(
        action=push,
        object_instances={'R':robot,'B':beam},
        action_object_instance_names=['R','B'],
        conditions=[in_assembly_c],
        condition_object_instance_names=[['B']],
        condition_values=[False]
    )
    #robot's hand must be empty
    push_ec2 = ExecutabilityCondition(
        action=push,
        object_instances={'R':robot,'B':beam,'T':thing},
        action_object_instance_names=['R','B'],
        conditions=[in_hand_c, Property('thing','T',Relation.IS_OF_SORT)],
        condition_object_instance_names=[['R','T'],['T']],
        condition_values=[True,True]
    )
    # must be at postion "near_to" target position to start a push action?
    push_ec3 = ExecutabilityCondition(
        action=push,
        object_instances={'R':robot,'B':beam,'C1':place_f, 'C2':place_f},
        action_object_instance_names=['R','B'],
        conditions=[location, near_to, assem_target_loc],
        condition_object_instance_names=[['R','C1'],['C1','C2'],['B','C2']],
        condition_values=[True,False,True]
    )
    # must share beam location to push
    push_ec4 = ExecutabilityCondition(
        action=push,
        object_instances={'R':robot,'B':beam,'C1':place_f, 'C2':place_f},
        action_object_instance_names=['R','B'],
        conditions=[location, location, Property('C1','C2',Relation.NOT_EQUAL)],
        condition_object_instance_names=[['R','C1'],['B','C2'],['C1','C2']],
        condition_values=[True,True,True]
    )
    # must be at a near_to location
    push_ec5 = ExecutabilityCondition(
        action=push,
        object_instances={'R':robot,'B':beam, 'C1':place_f},
        action_object_instance_names=['R','B'],
        conditions=[location, Property('near_to_location','C1',Relation.IS_OF_SORT)],
        condition_object_instance_names=[['R','C1'],['C1']],
        condition_values=[True,False]
    )
    push_action = Action(push,[push_c1, push_c2],[push_ec1,push_ec2,push_ec3,push_ec4, push_ec5])

    #!move_local action for control (servo if a move gets near to but not at target)
    move_local = ActionDefinition('move_local',[robot,place_f])
    ##move_local causes robot location to change
    ml_c1 = CausalLaw(
        action = move_local,
        object_instances={'R':robot,'C1':place_f},
        action_object_instance_names=['R','C1'],
        fluent_affected=location,
        fluent_object_instance_names=['R','C1'],
        fluent_value=True,
    )
    ##move_local can only be used from near_to positions
    ml_ec1 = ExecutabilityCondition(
        action=move_local,
        action_object_instance_names=['R','C2'],
        object_instances={'R':robot,'C1':near_to_loc,'C2':place_f},
        conditions=[location,near_to],
        condition_object_instance_names=[['R','C1'], ['C1','C2']],
        condition_values=[True,False]
    )
    #move_local only works from near_to locaitons
    ml_ec2 = ExecutabilityCondition(
        action=move_local,
        object_instances={'R':robot,'P1':place_f,'P2':place_f},
        action_object_instance_names=['R','P2'],
        conditions=[location, Property('near_to_location','P1',Relation.IS_OF_SORT)],
        condition_object_instance_names=[['R','P1'],['P1']],
        condition_values=[True,False],
    )
    ## cannot move_local to location you are already at
    ml_ec3 = ExecutabilityCondition(
        action=move_local,
        object_instances={'R':robot,'P1':place_f,'P2':place_f},
        action_object_instance_names=['R','P1'],
        conditions=[location, Property('P1','P2',Relation.EQUAL)],
        condition_object_instance_names=[['R','P2'],['P1','P2']],
        condition_values=[True,True],
    )

    move_local_action = Action(move_local,[ml_c1],[ml_ec1, ml_ec2, ml_ec3])

    # create fine res ALD
    #!ALD

    sorts = [robot,beam,link,in_m_end,angle_m_end,in_f_end,joint,beam_part,pin,thing,thing_part,ob,place_c,non_placement_loc,
    approach_loc,prerot_loc,through_loc,target_loc,assembly_loc,near_to_loc,input_locations,place_f,fine_res_sort,coarse_res_sort,
    thru_m, thru_f, angle_f]


    actions = [put_down_action,
            move_action,
            pick_up_action,
            assemble_cap_action,
            assemble_square_action,
            fasten_action,
            push_action,
            move_local_action]

    #example with 4 beams
    ALD = ActionLangSysDesc(
            sorts=sorts,
            statics=statics,
            state_constraints=state_constraints,
            inertial_fluents=fluents,
            actions=actions,
            domain_setup=[],
            goal_description=[],
            planning_steps = 1
        )

    return ALD
