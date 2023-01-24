from al_structures import *

SAVE_DIR = '/home/local/MTC_ORI_Collab/sparc_planning/sparc_files/'

#!sorts
robot = BasicSort('robot', ['rob0'])
thing = BasicSort('thing', [])
ob = SuperSort('object', [robot, thing])
grasp_mode = BasicSort('grasp_mode', ['dexterous', 'vacuum'])
place_c = BasicSort('place_c', ['input_area', 'intermediate_area', 'assembly_area'])
table_locations = BasicSort('table_locs', ['c1','c2','c3','c4','c5','c6']) #6 places in intermediate area for holding objects
non_placement_loc = BasicSort('non_placement_location',['above_input','above_intermediate','above_assembly'])
place_f = SuperSort('place_f', [table_locations,non_placement_loc])
# need sets to allow for more than one refined sort type using the component keyword
fine_res_sort = SuperSort('fine_res_sort',[place_f])
coarse_res_sort = SuperSort('coarse_res_sort',[place_c])

#!statics
next_to_c = Func('next_to_c', [place_c,place_c], FuncType.STATIC)
next_to_f = Func('next_to_f', [place_f,place_f], FuncType.STATIC)
#add component relation
component = Func('component',[coarse_res_sort,fine_res_sort], FuncType.STATIC)
                      
#!fluents
in_hand = Func('in_hand', [robot,thing], FuncType.FLUENT)
coarse_location = Func('loc_c', [ob,place_c], FuncType.FLUENT)
location = Func('loc_f', [ob,place_f], FuncType.FLUENT)
current_grasp_mode = Func('current_grasp_mode', [robot, grasp_mode], FuncType.FLUENT)
on = Func('on', [thing,thing], FuncType.FLUENT)
clear = Func('clear', [thing], FuncType.FLUENT)

#!state constraints
state_cons = []
## only place_f can be component of place_c
place_component_limit = StateConstraint(
    object_instances={'C1':place_f,'P1':place_c,},
    head=component,
    head_value=False,
    head_object_instance_names=['C1','P1'],
    conditions=[Property('place_c','P1',Relation.IS_OF_SORT),Property('place_f','C1',Relation.IS_OF_SORT)],
    condition_values=[True,False],
    condition_object_instance_names=[['P1'],['C1']]
)
state_cons.append(place_component_limit)

next_to_transivity = StateConstraint(
    object_instances={'C1':place_f,'C2':place_f},
    head=next_to_f,
    head_value=True,
    head_object_instance_names=['C1','C2'],
    conditions=[next_to_f],
    condition_object_instance_names=[['C2','C1']],
    condition_values=[True]
)
state_cons.append(next_to_transivity)

CWA_next_to_c = StateConstraint(
    object_instances={'P1':place_c,'P2':place_c},
    head=next_to_c,
    head_object_instance_names=['P1','P2'],
    head_value=False,
    conditions=[next_to_c],
    condition_object_instance_names=[['P1','P2']],
    condition_values=[False]
)
CWA_next_to_f = StateConstraint(
    object_instances={'C1':place_f,'C2':place_f},
    head=next_to_f,
    head_object_instance_names=['C1','C2'],
    head_value=False,
    conditions=[next_to_f],
    condition_object_instance_names=[['C1','C2']],
    condition_values=[False]
)
state_cons.append(CWA_next_to_c)
state_cons.append(CWA_next_to_c)

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
state_cons.append(next_to_f_c)

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
state_cons.append(loc_bridge_axiom)

object_permanence = StateConstraint(
    object_instances={'P1':place_f,'P2':place_f, 'T':thing},
    head=location,
    head_object_instance_names=['T','P1'],
    head_value=False,
    conditions=[location, Property('P1','P2',Relation.NOT_EQUAL)],
    condition_object_instance_names=[['T','P2'],['P1','P2']],
    condition_values=[True,True]
)
state_cons.append(object_permanence)

CWA_grasp_mode = StateConstraint(
    object_instances={'R':robot,'G1':grasp_mode,'G2':grasp_mode},
    head=current_grasp_mode,
    head_object_instance_names=['R','G1'],
    head_value=False,
    conditions=[current_grasp_mode, Property('G1','G2',Relation.NOT_EQUAL)],
    condition_object_instance_names=[['R','G2'],['G1','G2']],
    condition_values=[True,True]
)
state_cons.append(CWA_grasp_mode)

#objects on-top of another object share a location
shared_locations = StateConstraint(
    object_instances={'T1':thing,'T2':thing,'P1':place_f},
    head=location,
    head_object_instance_names=['T1','P1'],
    head_value=True,
    conditions=[location, on],
    condition_object_instance_names=[['T2','P1'],['T1','T2']],
    condition_values=[True,True]
)
state_cons.append(shared_locations)

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
##removes objects from on top of other objects
pu_c2 = CausalLaw(
    action=pick_up,
    object_instances={'R':robot,'T1':thing, 'T2':thing},
    action_object_instance_names=['R','T1'],
    fluent_affected=on,
    fluent_object_instance_names=['T1','T2'],
    fluent_value=False,
)
pu_c3 = CausalLaw(
    action=pick_up,
    object_instances={'R':robot,'T1':thing, 'T2':thing},
    action_object_instance_names=['R','T1'],
    conditions=[on],
    condition_object_instance_names=[['T1','T2']],
    condition_values=[True],
    fluent_affected=clear,
    fluent_object_instance_names=['T2'],
    fluent_value=True,
)
##cannot pick up unless clear
pu_ec1 = ExecutabilityCondition(
    action=pick_up,
    object_instances={'R':robot,'T1':thing},
    action_object_instance_names=['R','T1'],
    conditions=[clear],
    condition_object_instance_names=[['T1']],
    condition_values=[False],
)
##cannot pick up unless in same location
pu_ec2 = ExecutabilityCondition(
    action=pick_up,
    object_instances={'R':robot,'T1':thing, 'P1':place_f, 'P2':place_f},
    action_object_instance_names=['R','T1'],
    conditions=[location, location, Property('P1','P2',Relation.NOT_EQUAL)],
    condition_object_instance_names=[['T1','P1'],['T2','P2'],['P1','P2']],
    condition_values=[True, True, True],
)
pick_up_action = Action(pick_up,[pu_c1,pu_c2,pu_c3],[pu_ec1,pu_ec2])

#!move action (fine res)
move_f = ActionDefinition('move_f', [robot,place_f])
m_c1 = CausalLaw(
    action=move_f,
    object_instances={'R':robot,'P':place_f},
    action_object_instance_names=['R','P'],
    fluent_object_instance_names=['R','P'],
    fluent_affected=location,
    fluent_value=True
    )
#if the robot is holding an object the object will move with it
m_c2 = CausalLaw(
    action=move_f,
    object_instances={'R':robot,'P':place_f,'T':thing},
    action_object_instance_names=['R','P'],
    fluent_object_instance_names=['T','P'],
    fluent_affected=location,
    fluent_value=True,
    conditions=[in_hand],
    condition_values=[True],
    condition_object_instance_names=[['R','T']],
    )
##cannot move to location you are already at
m_ec1 = ExecutabilityCondition(
    action=move_f,
    object_instances={'R':robot,'P1':place_f,'P2':place_f},
    action_object_instance_names=['R','P1'],
    conditions=[location, Property('P1','P2',Relation.EQUAL)],
    condition_object_instance_names=[['R','P2'],['P1','P2']],
    condition_values=[True,True],
   )
## cannot move to location unless it is next to the current location
m_ec2 = ExecutabilityCondition(
    action=move_f,
    object_instances={'R':robot,'P1':place_f,'P2':place_f},
    action_object_instance_names=['R','P1'],
    conditions=[location, next_to_f],
    condition_object_instance_names=[['R','P2'],['P1','P2']],
    condition_values=[True,False],
   )
move_f_action = Action(move_f, [m_c1,m_c2], [m_ec1,m_ec2])

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
##puts objects onto other objects (modified for fine res places)
pd_c2 = CausalLaw(put_down,
    object_instances={'R':robot,'T1':thing,'T2':thing,'P1':place_f,'P2':place_f},
    action_object_instance_names=['R','T1'],
    fluent_object_instance_names=['T1','T2'],
    fluent_affected=on,
    fluent_value=True,
    conditions=[location,location,Property('P1','P2',Relation.EQUAL),clear],
    condition_object_instance_names=[['R','P1'],['T1','P2'],['P1','P2'],['T2']],
    condition_values=[True,True,True,True],
    )
# cannot put down objects in the new non-placement locations
pd_ec2 = ExecutabilityCondition(
    action = put_down,
    object_instances= {'R':robot, 'T':thing, 'C':place_f},
    action_object_instance_names=['R','T'],
    conditions= [location,Property('non_placement_location','C',Relation.IS_OF_SORT)],
    condition_object_instance_names= [['R','C'],['C']],
    condition_values= [True,True],
)
put_down_action = Action(put_down,[pd_c1,pd_c2],[pd_ec1])

#!change_grasp_mode action
cgm=ActionDefinition('change_grasp_mode',[robot,grasp_mode])
cgm_c1 = CausalLaw(cgm,
        fluent_affected=current_grasp_mode,
        fluent_value=True,
        fluent_object_instance_names=['R','G'],
        action_object_instance_names=['R','G'],
        object_instances={'R':robot,'G':grasp_mode},
        )
##cannot change to graspmode if already using it
cgm_ec1 = ExecutabilityCondition(
    cgm,
    object_instances={'R':robot,'G1':grasp_mode,'G2':grasp_mode},
    action_object_instance_names=['R','G1'],
    conditions=[current_grasp_mode,Property('G1','G2',Relation.EQUAL)],
    condition_values=[True,True],
    condition_object_instance_names=[['R','G1'],['G1','G2']],
    )
##cannot change grasp mode whilst holdiong an object
cgm_ec2 = ExecutabilityCondition(
    action= cgm,
    object_instances={'R':robot,'G':grasp_mode,'T':thing},
    action_object_instance_names=['R','G'],
    conditions=[in_hand],
    condition_values=[True],
    condition_object_instance_names=[['R','T']],
    )
cgm_action=Action(cgm,[cgm_c1],[cgm_ec1,cgm_ec2])

#!ALD
ALD = ActionLangSysDesc(
        sorts=[robot,
            thing,
            ob ,
            grasp_mode,
            place_c ,
            table_locations,
            non_placement_loc,
            place_f,
            fine_res_sort,
            coarse_res_sort],
        statics=[next_to_c,
                 next_to_f,
                 component],
        inertial_fluents=[in_hand,
                          coarse_location,
                          location,
                          current_grasp_mode,
                          on,
                          clear],
        actions=[put_down_action,
                 move_f_action,
                 pick_up_action,
                 cgm_action],
        state_constraints=state_cons,
        domain_setup=['holds(in_hand(rob0,textbook), true, 0).'],
        goal_description=[GoalDefinition(in_hand,['rob0','textbook'],False)],
        display_hints=['occurs.'], planning_steps=1)

ALD.to_sparc_program().save(SAVE_DIR+'robot_domain_fine.sp')