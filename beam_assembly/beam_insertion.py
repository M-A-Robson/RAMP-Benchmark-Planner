import copy
from typing import List, Dict, Tuple
from beam_assembly.beam_assembly_parser import Beam, BeamAssembly, BeamComponent
import numpy as np
import re
import logging
import trimesh

ACTION_SET = [
    'assemble_f_square',
    'assemble_f_cap',
    'assemble_f_through',
    'assemble_f_rotate',
    ]

def get_target_connection(
        beam_assembly:BeamAssembly,
        beam_to_insert:Beam,
        insertion_end:BeamComponent
        ) -> Tuple[BeamComponent, np.ndarray]:
    beam_connections = beam_assembly.get_beam_connections(beam_to_insert.name)
    target_beam = None
    for con in beam_connections:
        if con.element1 == beam_to_insert:
            if con.joint1 == insertion_end:
                target_component = con.joint2
                target_beam = con.element2
        else:
            if con.joint2 == insertion_end:
                target_component = con.joint2
                target_beam = con.element2
    if not target_beam:
        raise ValueError('No Target Beam found for this joint')
    target_beam_struct = target_beam.get_structure()
    target_comp_offsets = target_beam.get_component_offsets()
    target_component_index = target_beam_struct.index(target_component)
    target_pose = np.matmul(target_beam.origin, target_comp_offsets[target_component_index])
    return target_component, target_pose

def calculate_approach(
        beam_assembly:BeamAssembly,
        beam_to_insert:Beam,
        insertion_end:BeamComponent,
        approach_dist:float,
        assembly_action_target_pose:np.ndarray,
        cap:bool=False,
    ) -> np.ndarray:
    """calculates an offset pose target (approach locaiton) for beam motion before assembly action"""
    assert insertion_end in beam_to_insert.joints
    # get beam data
    beam_structure = beam_to_insert.get_structure()
    beam_comp_offsets = beam_to_insert.get_component_offsets()
    insertion_end_index = beam_structure.index(insertion_end) # should be first or last!
    insertion_end_target_pose = np.matmul(beam_to_insert.origin, beam_comp_offsets[insertion_end_index])
    # find connection data
    target_component, target_component_pose = get_target_connection(beam_assembly,beam_to_insert, insertion_end)
    # insertion axis should be target part x axis for square insertion
    insertion_axis = target_component_pose[:3,0]
    if cap:
        # insertion axis should be target part -x axis for cap insertion
        insertion_axis = -target_component_pose[:3,0]
        # for capping angle beams approach using open side of female part
        if target_component.is_male() and target_component.is_angle():
            insertion_axis = insertion_end_target_pose[:3,0]
 
    # calculate offset beam position
    approach_pose = copy.deepcopy(assembly_action_target_pose)
    approach_pose[:3,3] += insertion_axis*approach_dist

    return approach_pose

def calculate_insertion_poses(
        beam_assem:BeamAssembly,
        insertion_actions:List[str],
        approach_dist:float,
        through_dist:float,
    ) -> Dict[str,np.ndarray]:
    """
    Calculate poses to take beam to in order to complete a 
    set of fine resolution actions. For a coarse resolution action
    assemble(rob0, beam) -> 4 action combinations exist:

    1.  'occurs(move(rob0,beama),i)'
        'occurs(assemble_f_square(rob0,j3),i+1)'
    beam needs to be inserted along axis requiring an approach pose
    at "approach_dist" mm along beam axis from target pose.

    2.  'occurs(move(rob0,beama),i)'
        'occurs(assemble_f_cap(rob0,j7),i+1)'
    beam needs to be inserted across axis requiring an approach pose
    at "approach_dist" mm across beam axis from target pose, correctly
    offset based on the connecting beams.

    3.  'occurs(move(rob0,beama),i)'
        'occurs(assemble_f_square(rob0,j3),i+1)'
        'occurs(assemble_f_rotate(rob0,j3),i+2)'
    beam needs prerotate_pose based on square insertion with approach
    pose as (1).
        
    4.  'occurs(move(rob0,beama),i)'
        'occurs(assemble_f_through(rob0,j5),i+1)'
        'occurs(assemble_f_square(rob0,j3),i+2)'
    beam needs through pose as a target of the assemble_f_through action
    which replaces the approach pose for the assemble_square action by (1).
    Additionally the beams approach pose should be offset for the through
    action so that the beam end is through_dist mm from the part which the
    beam will be inserted through.

    Known limitations:    
    - cannot currently handle beams that both rotate and go through other beams,
    as we assume square insert then rotation.
    - cannot currently handle beams which pass through multiple other beams.

    Args:
            beam_assem (BeamAssembly): assembly description
            insertion_actions (List[str]): sparc actions for beam
                                           insertion at fine res 
    Returns:
        Dict[str:np.ndarray]: location_names, 4x4 pose for beam
    """
    
    # extract assembly actions and relevant components for each action
    actions = []
    components = []
    for action_str in insertion_actions:
        split = re.split('\(|\)|\,',action_str)
        # only care about relevant assembly actions
        if split[1] in ACTION_SET:
            actions.append(split[1])
            components.append(split[3])
    logging.info(components)
    # find beam by part names
    beam = beam_assem.get_beam_by_component_name(components[-1])
    # only update positions if not already calculated (default beam origin is I)
    if np.allclose(beam.origin, np.eye(4), 0.001):
        beam_assem.get_target_beam_positons()
    # add target pose to set
    poses = {f'{beam.name}t':beam.origin}

    insert_end = beam.get_component_by_name(components[-1])
    if 'assemble_f_through' in actions:
        #! only works for single through joint
        # todo extend to multiple through joints
        through_pose = calculate_approach(beam_assem, beam, insert_end, approach_dist, beam.origin)
        # calcualte offset from end to through component
        beam_component_offsets = beam.get_component_offsets()
        beam_structure = beam.get_structure()
        end_joint_point = beam_component_offsets[beam_structure.index(insert_end)][:3,3]
        through_joint_point = beam_component_offsets[beam_structure.index(beam.get_component_by_name(components[-2]))][:3,3]
        offset_dist = np.linalg.norm(end_joint_point-through_joint_point)
        # calculat new approach point with additional offset for through beam insertion action
        approach_pose = calculate_approach(beam_assem, beam, insert_end, offset_dist+through_dist, beam.origin)
        poses[f'{beam.name}a'] = approach_pose
        poses[f'{beam.name}_th'] = through_pose
        return poses
    
    if 'assemble_f_rotate' in actions:
        insert_end = beam.get_component_by_name(components[-2])
        target_component, target_pose = get_target_connection(beam_assem, beam, insert_end)
        beam_component_offsets = beam.get_component_offsets()
        beam_structure = beam.get_structure()
        end_joint_offset = beam_component_offsets[beam_structure.index(insert_end)] # beam origin to inserted part
        end_joint_pose = beam.origin@end_joint_offset
        #in-m-end y axis should align with target_connection (angle-f) x axis for correct fit
        insert_target_pose = np.eye(4)
        insert_target_pose[:3,3] = end_joint_pose[:3,3]
        # get angle phi between vector of beam 1 (non inserted end to inserted end) and target x_axis
        # cos(T)=(a.b)/(|a||b|)
        logging.info(end_joint_pose)
        logging.info(target_pose)
        phi = np.arccos(np.dot(end_joint_pose[:3,0], target_pose[:3,0]))
        # if -90<phi<90 then we want to approach from -x side of target
        if beam_assem.get_beam_by_component_name(target_component.name).flipped:
            phi += np.pi
        logging.info(np.degrees(phi))
        if np.linalg.norm(phi) < np.pi:
            insert_target_pose[:3,:3] = [[0,0,1],[0,1,0],[-1,0,0]]
        #if 180<phi<360 then we want to apprach from +x side of target
        else:
            insert_target_pose[:3,:3] = [[0,0,-1],[0,1,0],[1,0,0]]      

        
        prerotate_pose = insert_target_pose@np.linalg.inv(end_joint_offset)
        approach_pose = calculate_approach(beam_assem, beam, insert_end, approach_dist, prerotate_pose)

        poses[f'{beam.name}a'] = approach_pose
        poses[f'{beam.name}_pr'] = prerotate_pose
        return poses
    
    if 'assemble_f_cap' in actions:
        approach_pose = calculate_approach(beam_assem, beam, insert_end, approach_dist,beam.origin, cap = True)
        poses[f'{beam.name}a'] = approach_pose
        return poses
    
    # assumes square insert in no other cases caught
    approach_pose = calculate_approach(beam_assem, beam, insert_end, approach_dist, beam.origin)
    poses[f'{beam.name}a'] = approach_pose

    return poses


def visualise_poses(beam_assem:BeamAssembly, inserted_beam:Beam, insertion_poses:Dict[str,np.ndarray]):
    """visualises beams, moving target beam through named poses for visual check of pose calculations"""
    def create_display_scene(beam_assem:BeamAssembly)->trimesh.Scene:
        """modified trimesh scene for displaying beam assembly"""
        i = 0
        # create scene
        scene = trimesh.Scene()
        for beam in beam_assem.beams:
            comps = beam.get_structure()
            pos = beam.get_component_offsets()
            for p,c in zip(pos,comps):
                # load mesh
                mesh = c.load_model()
                # calculate component world pose
                t = np.matmul(beam.origin, p)
                scene.add_geometry(mesh, transform=t)
            i+=1
        return scene

    beam_index = beam_assem.beams.index(inserted_beam)
    #update nominal positions
    _ = beam_assem.get_target_beam_positons()

    for p,v in insertion_poses.items():
        logging.info(p)
        original_pose = copy.deepcopy(beam_assem.beams[beam_index].origin)
        beam_assem.beams[beam_index].origin = v
        scene = create_display_scene(beam_assem)
        scene.show()
        #reset pose
        beam_assem.beams[beam_index].origin = original_pose


def main():
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)

    from beam_assembly_parser import load_assembly_xml, load_beam_xml
    beams = load_beam_xml("example_beamset_2.xml")
    assem = load_assembly_xml(beams, "assembly_4.xml")
    # display default scene
    scene = assem.create_display_scene(colorise=True)
    scene.show()

    #! tests
    # beam5 (square)
    actions = ['occurs(assemble_f_square(rob0,J5-1),1)']
    poses = calculate_insertion_poses(assem,actions,0.05,0.2)
    for beam in beams:
        if beam.name == 'beam5':
            inserted_beam = beam
    visualise_poses(assem,inserted_beam,poses)
    
    actions = ['occurs(assemble_f_square(rob0,J5-3),1)']
    poses = calculate_insertion_poses(assem,actions,0.05,0.2)
    visualise_poses(assem,inserted_beam,poses)

    #beam8 (cap)
    actions = [
        'occurs(move(rob0,beam8a),0)',
        'occurs(assemble_f_cap(rob0,J8-1),1)',
    ]
    poses = calculate_insertion_poses(assem,actions,0.05,0.2)
    for beam in beams:
        if beam.name == 'beam8':
            inserted_beam = beam
    visualise_poses(assem,inserted_beam,poses)

    actions = [
        'occurs(move(rob0,beam8a),0)',
        'occurs(assemble_f_cap(rob0,J8-4),1)',
    ]
    poses = calculate_insertion_poses(assem,actions,0.05,0.2)
    visualise_poses(assem,inserted_beam,poses)

    #beam6 (rotated)
    actions = [
        'occurs(move(rob0,beam6),0)',
        'occurs(assemble_f_square(rob0,J6-3),1)',
        'occurs(assemble_f_rotate(rob0,J6-3),2)',
    ]
    poses = calculate_insertion_poses(assem,actions,0.05,0.2)
    for beam in beams:
        if beam.name == 'beam6':
            inserted_beam = beam
    visualise_poses(assem,inserted_beam,poses)

    actions = [
        'occurs(move(rob0,beam6),0)',
        'occurs(assemble_f_square(rob0,J6-1),1)',
        'occurs(assemble_f_rotate(rob0,J6-1),2)',
    ]
    poses = calculate_insertion_poses(assem,actions,0.05,0.2)
    visualise_poses(assem,inserted_beam,poses)


if __name__ == '__main__':
    main()