"""
    Copyright (C) 2023 The Manufacturing Technology Centre
    Author: Mark Robson

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

import os
from sparc_planning.src.al_structures import Sort, BasicSort
from beam_assembly.beam_assembly_parser import BeamAssembly, ElementType, load_assembly_xml, load_beam_xml
from typing import List, Tuple
import logging

def extract_component_sorts(data:dict)->List[Sort]:
    """turns dict output from Beam.get_sparc_representation()[1]
    into Sorts. This is seperated out as it is necessary to collate
    component data for all beams in the assembly before parsing to sparc.
    Args:
        data (dict): beam component data
    Returns:
        list[Sort]: component_sorts
    """

    in_m_end_instances = []
    component_data = []
    for key in data:
        if len(data[key])>0:
            sort_name:str = key.replace('-','_')
            # fudge to collate in_m_end objects together despite different naming in xml for
            # parts with additional feet in mesh
            if sort_name == 'in_m_end_feet':
                in_m_end_instances += data[key]
                continue
            if sort_name == 'in_m_end':
                in_m_end_instances += data[key]
                continue
            component_data.append(BasicSort(sort_name, data[key]))

    if len(in_m_end_instances) > 0:
        component_data.append(BasicSort('in_m_end', in_m_end_instances))

    logging.debug(f'Extracted Sorts: {[s.name for s in component_data]}')
    return component_data

def create_sparc_data(assem:BeamAssembly) -> Tuple[List[Sort],List[str],List[Sort],List[str],]:
    """Generate coarse and fine domain data for this assembly to be used in planning

    Args:
        Assem (BeamAssembly): assembly description

    Returns:
        List[Sort]: coarse res sorts
        List[str]: coarse res domain description
        List[Sort]: fine res sorts
        List[str]: fine res domain description
    """
    # at course resolution we want:
    # - beams
    # - beam to beam connection_c and type

    # at fine resolution we want:
    # for subset of beams to reason about:
    # - beams
    # - beam component sorts
    # - pin component sorts (component(pin,pin))
    # - component relations (connections) internal
    # - conenction_f relations (connections) external
    # - associated location sorts (input, approach, target,
    #   (optional) pre-rotate, (optional) through)
    #   near_to locations and next_to_f relations

    beams = assem.get_beams()
    # extract beam connections (coarse domain description)
    beam_connections = [conn.to_sparc_coarse() for conn in assem.connections]
    fine_beam_connections = [conn.to_sparc_fine() for conn in assem.connections]
    # create a pin for each conneciton
    pin_sort = BasicSort('pin',[f'p{i}' for i in range(1,len(beam_connections)+1)])
    base_statement = f'base({assem.base.name}).'
    beam_connections.append(base_statement)
    fine_statics = [f'component({pin},{pin}).' for pin in pin_sort.instances]
    beam_names = []
    fine_statics += fine_beam_connections
    components = {}
    for b in beams:
        # extract beam data
        beam_name, beam_components, beam_internal_connections, component_relations = b.get_sparc_representation()
        beam_names.append(beam_name)
        logging.debug(f'Extracting data for beam: {beam_name}')
        fine_statics += beam_internal_connections
        fine_statics += component_relations
        # collate components into a single dict (by component type)
        for k in beam_components:
            v = beam_components[k]
            if k in components.keys():
                components[k] = components[k]+(v) # join lists
            else: components[k] = v
    # extract beam component sorts
    component_sorts = extract_component_sorts(components)
    beam_sort = BasicSort('beam', beam_names)

    coarse_sorts = [beam_sort,pin_sort]

    object_names = beam_names + pin_sort.instances
    input_locations = BasicSort('input_location',[f'{o}i' for o in object_names])
    target_locations = BasicSort('target_location',[f'{o}t' for o in object_names])
    approach_locations = BasicSort('approach_location',[f'{o}a' for o in object_names])
    fine_statics += [f'assem_target_loc({thing},{thing}t).' for thing in object_names]
    fine_statics += [f'assem_approach_loc({thing},{thing}a).' for thing in object_names]
    
    # through and pre-rotation locations
    tl_instances = []
    prl_instances = []
    through_and_prerotate_location_statics = []
    for beam in beams:
        beam_component_types = beam.get_component_type_list()
        if ElementType.THRU_M.value in beam_component_types:
            # add through location
            tl_instances.append(f'{beam.name}_th')
            through_and_prerotate_location_statics.append(f'beam_through_loc({beam.name},{beam.name}_th).')
        if ElementType.ANGLE_M_END.value in beam_component_types:
            # add pre_rotation location
            prl_instances.append(f'{beam.name}_pr')
            through_and_prerotate_location_statics.append(f'beam_prerotate_loc({beam.name},{beam.name}_pr).')
    through_locations = BasicSort('through_location',tl_instances)
    prerotate_locations = BasicSort('prerot_location',prl_instances)
    fine_statics += through_and_prerotate_location_statics
    # add near_to locations for each target locatiton (these atomatically get assigned component relations)
    # TODO may need to revise when better control is added for actions and more near_to relations are neded
    location_names = tl_instances + prl_instances + input_locations.instances + target_locations.instances + approach_locations.instances
    near_to_locations = BasicSort('near_to_location',[f'nt_{o}' for o in location_names])
    fine_statics += [f'near_to(nt_{o},{o}).' for o in location_names]
    
    locations = [input_locations,
                 target_locations,
                 approach_locations,
                 through_locations,
                 prerotate_locations,
                 near_to_locations]
    
    fine_sorts = [beam_sort, pin_sort] + component_sorts + locations

    # add component relations for locations
    fine_statics += [f'component(input_area,{i}).' for i in input_locations.instances]
    fine_statics += [f'component(assembly_area,{i}).' for i in target_locations.instances] 
    fine_statics += [f'component(assembly_area,{i}).' for i in approach_locations.instances] 
    fine_statics += [f'component(assembly_area,{i}).' for i in through_locations.instances] 
    fine_statics += [f'component(assembly_area,{i}).' for i in prerotate_locations.instances] 
    # add next_to_f mapping for fine locations
    # approach -> through/prerot -> target
    next_to_f_str = []
    visited_targets = []
    for inst in prerotate_locations.instances:
        target = inst[:-3]+'t'
        approach = inst[:-3]+'a'
        next_to_f_str.append(f"next_to_f({inst},{target}).")
        next_to_f_str.append(f"next_to_f({inst},{approach}).")
        visited_targets.append(target)
    for inst in through_locations.instances:
        target = inst[:-3]+'t'
        approach = inst[:-3]+'a'
        next_to_f_str.append(f"next_to_f({inst},{target}).")
        next_to_f_str.append(f"next_to_f({inst},{approach}).")
        visited_targets.append(target)
    for inst in target_locations.instances:
        if inst not in visited_targets:
            approach = inst[:-1]+'a'
            next_to_f_str.append(f"next_to_f({inst},{approach}).")
    # approach and above_assembly_area
    next_to_f_str += [f'next_to_f(above_assembly_area,{i}).' for i in approach_locations.instances]
    #target and near_to_target locations with above_assembly_area
    next_to_f_str += [f'next_to_f(above_assembly_area,{i}).' for i in target_locations.instances]
    next_to_f_str += [f'next_to_f(above_assembly_area,{i}).' for i in near_to_locations.instances if i[-1] != 'i']
    next_to_f_str += [f'next_to_f(above_input_area,{i}).' for i in near_to_locations.instances if i[-1] == 'i']
    #inputs and above_input_area
    next_to_f_str += [f'next_to_f(above_input_area,{i}).' for i in input_locations.instances]
    
    fine_statics += next_to_f_str

    return coarse_sorts, beam_connections, fine_sorts, fine_statics


def main():

    logger = logging.getLogger()
    logger.setLevel(logging.INFO)

    beams = load_beam_xml(os.path.join(os.environ['PLANNER_PATH'], "example_beamset_latest.xml"))
    assem = load_assembly_xml(beams, os.path.join(os.environ['PLANNER_PATH'], "assembly_latest.xml"))
    
    scene = assem.create_display_scene(colorise=True)
    scene.show()

    coarse_sorts, coarse_statics, fine_sorts, fine_statics = create_sparc_data(assem)

    with open(os.path.join(os.environ['PLANNER_PATH'], 'example_latest_output.sp'), 'w') as file:
        header = [
            r'%%%%%%%%%%%%%%%%%%%%%%%%%%%%%','\n',
            r'%% AutoGenerated Beam Description file','\n',
            r'%% Author: MARK ROBSON 2023','\n',
            r'%% for example purposes only','\n',
            r'%%%%%%%%%%%%%%%%%%%%%%%%%%%%%','\n','\n']
        file.writelines(header + [r'% coarse data','\n'])
        for f in coarse_sorts:
            file.writelines([f.to_sparc(),'\n'])
        file.writelines(coarse_statics)
        file.writelines(['\n',r'% fine data','\n'])
        for f in fine_sorts:
            file.writelines([f.to_sparc(),'\n'])
        for f in fine_statics:
            file.write(f)

if __name__ == "__main__":
    main()
