from al_structures import *

SAVE_DIR = '/home/local/MTC_ORI_Collab/sparc_planning/sparc_files/'

#!sorts
robot = BasicSort('robot', ['rob0'])
#add beam and pin sorts (simple 4 beam set)
beam = BasicSort('beam',['b1','b2','b3','b4'])
link = BasicSort('link',['L1','L2','L3','L4'])
#!only end joints in this example set
in_m_end = BasicSort('in_m_end',['J3','J4','J7','J8'])
in_f_end = BasicSort('in_f_end',['J1','J2','J5','J6'])
end_joint = SuperSort('end_joint', [in_m_end,in_f_end])
beam_part = SuperSort('beam_part',[link,end_joint]) #mid_joint
pin = BasicSort('pin',['p1','p2','p3','p4'])
thing = SuperSort('thing', [beam,pin])
thing_part = SuperSort('thing_part', [beam_part, pin]) # graspable things
ob = SuperSort('object', [robot, thing])
place_c = BasicSort('place', ['input_area', 'intermediate_area', 'assembly_area'])
non_placement_loc = BasicSort('non_placement_location',['above_input','above_intermediate','above_assembly'])
# approach locations from xml extraction
approach_loc = BasicSort('approach_location', ['b1a','b2a','b3a','b4a','p1a','p2a','p3a','p4a'])
#through and pre-rotate locations from beam xml extraction
prerot_loc = BasicSort('prerot_location', []) #! no through or rotate beams in this set
through_loc = BasicSort('through_location', [])
# insert locatons for beams and pins from xml extraction
target_loc = BasicSort('target_location', ['b1t','b2t','b3t','b4t','p1t','p2t','p3t','p4t'])
# add a near_to location for all fine locations
near_to_loc = BasicSort('near_to_location', ['nt_b1a','nt_b2a','nt_b3a','nt_b4a','nt_p1a','nt_p2a',
                                             'nt_p3a','nt_p4a','nt_b1t','nt_b2t','nt_b3t','nt_b4t',
                                             'nt_p1t','nt_p2t','nt_p3t','nt_p4t','nt_above_input',
                                             'nt_above_intermediate','nt_above_assembly'])
place_f = SuperSort('place_f', [approach_loc,target_loc,non_placement_loc, near_to_loc]) # through_loc, pre_rotate_loc
# need sets to allow for more than one refined sort type using the component keyword
fine_res_sort = SuperSort('fine_res_sort',[place_f])
coarse_res_sort = SuperSort('coarse_res_sort',[place_c])

#!statics
next_to_c = Func('next_to_c', [place_c,place_c], FuncType.STATIC)
next_to_f = Func('next_to_f', [place_f,place_f], FuncType.STATIC)
# add component relation
component = Func('component',[coarse_res_sort,fine_res_sort], FuncType.STATIC)
# define close locations for recovery/motion control
near_to = Func('near_to', [near_to_loc,place_f])
# add beam domain statics with refinements
#TODO sort out relationships between f and c for fits_into and fits_through
fits_into_c = Func('fits_into_c', [beam,beam], FuncType.STATIC)
fits_into_f = Func('fits_into_f', [beam_part,beam_part], FuncType.STATIC)
fits_through_c = Func('fits_through_c',[beam,beam],FuncType.STATIC)
fits_through_f = Func('fits_through_f',[beam_part,beam_part],FuncType.STATIC)

is_capped_by = Func('is_capped_by',[beam,beam,beam],FuncType.STATIC)
# new fine res statics of beam domain
## beam part association
connected_to = Func('connected_to',[beam_part,beam_part])
between = Func('between',[beam_part,beam_part,beam_part])
## assembly location structure
assem_target_loc = Func('assem_target_loc', [thing, target_loc])
assem_approach_loc = Func('assem_approach_loc', [thing, approach_loc])
#! no rotation or through beams in this set
beam_prerotate_loc = Func('beam_prerotate_loc', [beam, prerot_loc])
beam_through_loc = Func('beam_through_loc', [beam, through_loc])

statics = [next_to_c,next_to_f,component,near_to,fits_into_c,
           fits_into_f,fits_through_c,fits_through_f,is_capped_by,
           connected_to,between, assem_approach_loc, assem_target_loc] 
        # beam_through_loc, beam_prerotate_loc

#!fluents
in_hand_c = Func('in_hand_c', [robot,thing], FuncType.FLUENT)
in_hand_f = Func('in_hand_f', [robot,thing_part], FuncType.FLUENT)
coarse_location = Func('loc_c', [ob,place_c], FuncType.FLUENT)
location = Func('loc_f', [ob,place_f], FuncType.FLUENT)
#current_grasp_mode = Func('current_grasp_mode', [robot, grasp_mode], FuncType.FLUENT)
#on = Func('on', [thing,thing], FuncType.FLUENT)
#clear = Func('clear', [thing], FuncType.FLUENT)
# add beam domain fluents
in_assembly = Func('in_assembly',[beam],FuncType.FLUENT)
supported = Func('supported',[beam], FuncType.FLUENT)
fastened = Func('fastened',[beam,beam,pin], FuncType.FLUENT)

fluents = [in_hand_c,in_hand_f,location,in_assembly,supported,fastened]

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
    condition_object_instance_names=[['R','TP'],['TP','T']],
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
## beams supported if either end can be joined to another beam
support_constraint = StateConstraint(
    object_instances={'B1':beam,'B2':beam},
    head=supported,
    head_object_instance_names=['B1'],
    head_value=True,
    conditions=[in_assembly,fits_into_c],
    condition_object_instance_names=[['B2'],['B1','B2']],
    condition_values=[True,True]
)
support_constraint2 = StateConstraint(
    object_instances={'B1':beam,'B2':beam},
    head=supported,
    head_object_instance_names=['B1'],
    head_value=True,
    conditions=[in_assembly,fits_into_c],
    condition_object_instance_names=[['B2'],['B2','B1']],
    condition_values=[True,True]
)
                      
##TODO rules govenerning fit interactions
##TODO do we need these static rules?
##TODO fits must be defined by designer anyway?
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
    condition_object_instance_names=[['D1','D2'],['D1','B1'],['D2','B2']],
    condition_values=[True,True,True]
))
state_constraints.append(StateConstraint(
    object_instances={'B1':beam,'D1':beam_part,'B2':beam,'D2':beam_part},
    head=fits_through_c,
    head_value=True,
    head_object_instance_names=['B1','B2'],
    conditions=[fits_through_f,component, component],
    condition_object_instance_names=[['D1','D2'],['D1','B1'],['D2','B2']],
    condition_values=[True,True,True]
))

#TODO action refinements and new actions at fine resolution
#removed on and clear as we can assume these for beam domain (no object stacking)
#!pick_up action
pick_up = ActionDefinition('pick_up', [robot, thing])
##puts thing into robots hand
pu_c1 = CausalLaw(
    action=pick_up,
    object_instances={'R':robot,'T':thing_part},
    action_object_instance_names=['R','T'],
    fluent_affected=in_hand_f,
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
    condition_object_instance_names=[['rob0','T2'],['T2']],
    condition_values=[True, True],
)
pick_up_action = Action(pick_up,[pu_c1,],[pu_ec2, pu_ec3])

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

## TODO - think about fastenening 
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
    condition_object_instance_names=[['R','P1'],['B1','B2','P']],
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

#!change_grasp_mode action
# cgm=ActionDefinition('change_grasp_mode',[robot,grasp_mode])
# cgm_c1 = CausalLaw(cgm,
#         fluent_affected=current_grasp_mode,
#         fluent_value=True,
#         fluent_object_instance_names=['R','G'],
#         action_object_instance_names=['R','G'],
#         object_instances={'R':robot,'G':grasp_mode},
#         )
# ##cannot change to graspmode if already using it
# cgm_ec1 = ExecutabilityCondition(
#     cgm,
#     object_instances={'R':robot,'G1':grasp_mode,'G2':grasp_mode},
#     action_object_instance_names=['R','G1'],
#     conditions=[current_grasp_mode,Property('G1','G2',Relation.EQUAL)],
#     condition_values=[True,True],
#     condition_object_instance_names=[['R','G1'],['G1','G2']],
#     )
# ##cannot change grasp mode whilst holdiong an object
# cgm_ec2 = ExecutabilityCondition(
#     action= cgm,
#     object_instances={'R':robot,'G':grasp_mode,'T':thing},
#     action_object_instance_names=['R','G'],
#     conditions=[in_hand],
#     condition_values=[True],
#     condition_object_instance_names=[['R','T']],
#     )
# cgm_action=Action(cgm,[cgm_c1],[cgm_ec1,cgm_ec2])

#!assemble action
assemble = ActionDefinition('assemble',[robot,beam])
asem_c1 = CausalLaw(
    action=assemble,
    fluent_affected=in_assembly,
    fluent_value=True,
    object_instances={'R':robot,'B':beam},
    action_object_instance_names=['R','B'],
    fluent_object_instance_names=['B'],
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
    object_instances={'R':robot,'B1':beam},
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
assemble_action = Action(assemble,[asem_c1],[asem_ec1,asem_ec2,asem_ec3,asem_ec5,asem_ec6,asem_ec7,asem_ec8])

#!Fasten Action
fasten = ActionDefinition('fasten',[robot,beam,beam,pin])
f_c1 = CausalLaw(
    action=fasten,
    fluent_affected=fastened,
    fluent_value=True,
    object_instances={'R':robot,'B1':beam,'B2':beam,'P1':pin},
    action_object_instance_names=['R','B1','B2','P1'],
    fluent_object_instance_names=['B1','B2','P1'],
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
fasten_action = Action(fasten,[f_c1],[f_ec1,f_ec2,f_ec3])

#!ALD
# example with 5 beams
ALD = ActionLangSysDesc(
        sorts=[robot,beam,pin,thing,ob,place],
        statics=statics,
        state_constraints=state_constraints,
        inertial_fluents=fluents,
        actions=[put_down_action,move_action,pick_up_action,assemble_action,fasten_action],
        domain_setup=['holds(in_assembly(b1),true,0).','holds(location(b1,assembly_area),true,0).','holds(location(b2,input_area),true,0).',
                      'holds(location(b3,input_area),true,0).','holds(location(b4,input_area),true,0).','holds(location(rob0,input_area),true,0).',
                      'next_to(input_area,intermediate_area).','holds(location(b5,input_area),true,0).',
                      'next_to(assembly_area,intermediate_area).','fits_into(b2, b1).','fits_into(b3, b1).','fits_into(b3, b4).',
                      'fits_into(b2, b4).', 'fits_into(b5,b2).', 'fits_into(b5,b3).'],
        goal_description=[GoalDefinition(in_assembly,['b4'],True)],
        display_hints=['occurs.'], planning_steps=25)

ALD.complete_domain_setup_fluent(in_hand,False) #assert robot hand is empty
ALD.complete_domain_setup_fluent(clear,True) #assert objects are accessible

ALD.to_sparc_program().save(SAVE_DIR+'beam_domain_coarse.sp')
