import os
from sparc_planning.src.al_structures import *

SAVE_DIR = os.path.join(os.environ['PLANNER_PATH'], '/sparc_planning/sparc_files/')

def generate_abstract_beam_domain():
    #!sorts
    beam = BasicSort('beam',[])

    #!statics
    fits_into = Func('fits_into_c', [beam,beam], FuncType.STATIC)
    fits_through = Func('fits_through_c',[beam,beam],FuncType.STATIC)
    is_capped_by = Func('is_capped_by',[beam,beam,beam],FuncType.STATIC)
    base = Func('base',[beam],FuncType.STATIC)

    statics = [fits_into,fits_through,is_capped_by,base]

    #!fluents
    in_assembly = Func('in_assembly_c',[beam],FuncType.FLUENT)
    supported = Func('supported_c',[beam], FuncType.FLUENT)

    fluents = [in_assembly,supported]

    #!state constraints
    state_constraints=[]

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

    
    #!assemble action
    assemble = ActionDefinition('assemble',[beam])
    #causes in assembly
    asem_c1 = CausalLaw(
        action=assemble,
        fluent_affected=in_assembly,
        fluent_value=True,
        object_instances={'B':beam},
        action_object_instance_names=['B'],
        fluent_object_instance_names=['B'],
    )
    
    # cannot add beam if both cap beams are already in place
    asem_ec2 = ExecutabilityCondition(
        action= assemble,
        object_instances={'B1':beam,'B2':beam,'B3':beam},
        action_object_instance_names=['B1'],
        conditions=[in_assembly,in_assembly,is_capped_by,
                    Property('B2','B3',Relation.NOT_EQUAL)],
        condition_values=[True,True,True,True],
        condition_object_instance_names=[['B2'],['B3'],['B1','B2','B3'],['B2','B3']],
    )
    #cannot add beam if another beam which needs to pass through the beam is alread in the assembly
    asem_ec3 = ExecutabilityCondition(
        action= assemble,
        object_instances={'B1':beam,'B2':beam},
        action_object_instance_names=['B1'],
        conditions=[in_assembly,fits_through],
        condition_values=[True,True],
        condition_object_instance_names=[['B2'],['B2','B1']],
    )
    #cannot add a beam which would cap another beam if the beams to be capped are not yet in the assembly
    asem_ec5 = ExecutabilityCondition(
        action= assemble,
        object_instances={'B1':beam,'B2':beam,'B3':beam},
        action_object_instance_names=['B1'],
        conditions=[in_assembly,in_assembly,is_capped_by,
                    Property('B2','B3',Relation.NOT_EQUAL)],
        condition_values=[False,True,True,True,True],
        condition_object_instance_names=[['B2'],['B3'],['B2','B1','B3'],['B2','B3']],
    )
   
    #cannot add beams already in the assembly
    asem_ec7 = ExecutabilityCondition(
        action=assemble,
        object_instances={'B':beam},
        action_object_instance_names=['B'],
        conditions=[in_assembly],
        condition_object_instance_names=[['B']],
        condition_values=[True],
    )
    #beams must be supported to add them
    asem_ec8 = ExecutabilityCondition(
        action=assemble,
        object_instances={'B':beam},
        action_object_instance_names=['B'],
        conditions=[supported],
        condition_object_instance_names=[['B']],
        condition_values=[False],
    )
   
    # assemble_action = Action(assemble,[asem_c1],[asem_ec1,asem_ec2,asem_ec3,asem_ec5,asem_ec6,asem_ec7,asem_ec8,asem_ec9])
    assemble_action = Action(assemble,
                             [asem_c1],
                             [asem_ec2,asem_ec3,asem_ec5,
                              asem_ec7,asem_ec8])

    
    #!ALD
    ALD = ActionLangSysDesc(
            sorts=[beam],
            statics=statics,
            state_constraints=state_constraints,
            inertial_fluents=fluents,
            actions=[assemble_action],
            domain_setup=[],
            goal_description=[],
            planning_steps=1)

    return ALD
