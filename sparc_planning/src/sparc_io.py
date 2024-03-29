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

from __future__ import annotations
import subprocess
from dataclasses import dataclass, field
import os
from typing import List, Optional, Tuple
from sortedcollections import OrderedSet
import re
import os

SPARC_LOC = os.environ['SPARC_PATH']

@dataclass(frozen=True)
class SparcState:
    statics : List[str] # things that don't change
    fluents : List[str] # things that change
    occurs : List[str] # planned actions at this timestep
    history : List[str] # things that actually happened at this timestep(hpd(x))
    goal_reached : bool
    time_step : int

@dataclass
class SparcProg:
    constants: List[str] = field(default_factory=list)
    sorts: List[str] = field(default_factory=list)
    predicates: List[str] = field(default_factory=list)
    rules: List[str] = field(default_factory=list)
    display: Optional[List[str]] = None

    def save(self, filename:str) -> None:
        """save to sparc file"""
        if (os.path.exists(filename)):
            os.remove(filename)
        header = [
            r'%%%%%%%%%%%%%%%%%%%%%%%%%%%%%','\n',
            r'%% AutoGenerated SPARC file','\n',
            r'%% Author: MARK ROBSON 2023','\n',
            r'%%%%%%%%%%%%%%%%%%%%%%%%%%%%%','\n','\n']
        with open(filename, 'x') as f:
            f.writelines(header)
            f.writelines(line + '\n' for line in self.constants)
            f.write('\nsorts\n')
            f.writelines(line + '\n' for line in self.sorts)
            f.write('\npredicates\n')
            f.writelines(line + '\n' for line in self.predicates)
            f.write('\nrules\n')
            f.writelines(line + '\n' for line in self.rules)
            # display is optional
            if self.display:
                f.write('\ndisplay\n')
                f.writelines(line + '\n' for line in self.display)
    
    @staticmethod
    def load(filename:str) -> SparcProg:
        """load ASP from sparc file"""
        # extract lines from file removing new lines and leading whitespace
        with open(filename, 'r') as f:
            lines = [line.rstrip().lstrip() for line in f]
        # slit into sections and clean up
        data = {'constants':[],'sorts':[],'predicates':[],'rules':[],'display':[]}
        i = 0
        for line in lines:
            #skip empty lines
            if len(line) == 0:
                continue 
            # skip comment lines
            if line[0] == '%':
                continue
            # update key
            if i<4:
                if list(data.keys())[i+1] in line:
                    i += 1
                    continue
            # add to data
            data[list(data.keys())[i]].append(line)
        # unpack into ASP object
        return SparcProg(**data)
    
    def __add__ (self, asp:SparcProg) -> SparcProg:
        """combine ASP objects, removes duplicate lines"""
        return SparcProg(
            constants=[*OrderedSet([*self.constants,*asp.constants])],
            sorts=[*OrderedSet([*self.sorts,*asp.sorts])],
            predicates=[*OrderedSet([*self.predicates,*asp.predicates])],
            rules=[*OrderedSet([*self.rules,*asp.rules])],
            display=[*OrderedSet([*self.display,*asp.display])],
            )
        
    def expand_sorts(self):
        """SPARC allows for shortening of sort definitions but in some cases we
        need to expand these out (e.g. for searching through sorts for object 
        instances when zooming """
        # assuming constants have form:
        # '#const name = value.'
        # extract name value pairs
        cv_pairs = {}
        for const in self.constants:
            c,v = re.split('\=', const[7:-1])
            cv_pairs[c.rstrip()] = v.lstrip()
        # check through sorts for expansions
        updated_sorts = []
        for sort in self.sorts:
            sort_name, sort_def = re.split(' \= ', sort)
            # check for '..m' pattern where m is a constant (defined with a name)
            m = re.search('\.\.\D+', sort_def)
            if m:
                # extract constant name
                const_name = sort_def[m.span()[0]+2:m.span()[1]]
                # remove trailing '.' if present
                if const_name[-1] == '.': const_name = const_name[:-1]
                # replace constant with value
                sort_def = sort_def[:m.span()[0]+2] \
                    + cv_pairs[const_name] \
                    + sort_def[m.span()[1]:]
            # check for 'n..' pattern where n is a constant
            n = re.search('\D+\.\.', sort_def)
            if n:
                # extract constant name
                const_name = sort_def[n.span()[0]:n.span()[1]-2]
                # remove trailing '.' if present
                if const_name[-1] == '.': const_name = const_name[:-1]
                # replace constant with value
                sort_def = sort_def[:m.span()[0]] \
                        + cv_pairs[const_name] \
                        + sort_def[m.span()[1]-2:]
            # find 'n..m' pattern with numberical n and m
            n_m = re.search('[0-9]+\.\.[0-9]+', sort_def)
            if n_m:
                start_ind, end_ind = n_m.span()
                start_num, end_num = re.findall('[0-9]+', sort_def[start_ind:end_ind])
                sort_def = sort_def[:start_ind] +'{' \
                    + str([i for i in range(int(start_num),int(end_num)+1)])[1:-1] \
                    + '}'+ sort_def[end_ind:]
            # find [b][{1,2,n}] pattern (which we have expanded from [b][1..n])
            b_pat = re.search('\[\w+\]\[\{.+\}\]', sort_def)
            if b_pat:
                values = re.findall('\w+', sort_def[b_pat.span()[0]:b_pat.span()[1]])
                out_values = [values[0]+v for v in values][1:]
                sort_def = sort_def[:b_pat.span()[0]] + '{' \
                    + ', '.join(out_values) +'}.'
            # add expanded sort to set, or original sort if no changes made
            updated_sorts.append(sort_name + ' = ' + sort_def)
        # override original sort data
        #print(updated_sorts)
        self.sorts = updated_sorts
                

# helper functions
def collect_sparc_files(files:list[str]) -> SparcProg:
    """fuse together multiple .sp files into one python object"""
    data = []
    # collects files into list of python objects
    for f in files:
        data.append(SparcProg.load(f))
    # combine files uses ordered set to screen out duplicate lines
    combined = SparcProg()
    for asp in data:
        combined += asp
    return combined

def parse_sparc_ouput(output_file:str='sparc.out') -> list[list[str]]:
    """extracts lines from sparc results file and cleans up notation and newlines
    returns a list of answer sets where each answer set is a list of strings"""
    with open(output_file, 'r') as f:
        lines = f.readlines()
        lines = [line.strip('?- ')for line in lines]
        a_sets = [line.split(', ') for line in lines]
        for a_s in a_sets:
            a_s[0] = a_s[0][1:]
            a_s[-1] = a_s[-1][:-2]
        answer_sets = [a for a in a_sets if a != ['']]
    return answer_sets

def extract_states_from_answer_set(answer_set:List[str],
                                   min_time_step:int = 0,) -> Tuple[List[SparcState], List[str]]:
    """extracts each state from answer set and planned actions
    Args:
        answer_set (List[str]): answer set parsed from sparc output
    Returns:
        list[SparcState]: states by timestep
        List[str]: planned action to take at each timestep format 'occurs(A(x1,x2..xn))
    """
    fluents = []
    statics = []
    occurs = []
    hpd = []
    goal = []
    for l in answer_set:
        #ignore planning stuff and things that don't happen
        if ('something_happened'in l) or ('-occurs'in l): 
            continue
        if ('hpd' in l): hpd.append(l)
        if ('val' in l) or ('holds' in l): fluents.append(l)
        elif 'occurs' in l: occurs.append(l)
        elif ('success'in l) or ('goal' in l): goal.append(l)
        else: statics.append(l+'.')

    # find the largest step value
    # updated to use regex to provide more general solution for max_time_step greater than 10
    max_time_step = max([int(re.findall('\d+', f)[-1]) for f in fluents])
    states = []
    actions = []
    for i in range(min_time_step,max_time_step+1):
        # find fluents and occurs statements for this time step
        flu = [fluent+'.' for fluent in fluents if int(re.findall('\d+', fluent)[-1])==i]
        occurs_t = [occ for occ in occurs if (int(re.findall('\d+', occ)[-1])==i) and (occ[0]!='-')]
        actions += occurs_t
        hpd_t = [h for h in hpd if int(re.findall('\d+', h)[-1])<=i]
        res = True if (f'goal({i+1})' in goal) else False
        states.append(SparcState(statics,flu,occurs_t,hpd_t,res,i))

    return states, actions 

def run_sparc(sparc_file:str = './sparc_files/program.sp', output_file:str='sparc.out', sparc_location=SPARC_LOC):
    """runs SPARC to generate answerset file with all answersets
    requires SPARC install (https://github.com/iensen/sparc)
    Args:
        sparc_file (str, optional): input sparc file. Defaults to './sparc_files/program.sp'.
        output_file (str, optional): saved output. Defaults to 'sparc.out'.
        sparc_locaiton (str, optional): location of sparc.jar defaults to os.environ['SPARC_PATH']
    """
    command = f'java -jar {sparc_location}/sparc.jar {sparc_file} -A >{output_file}'
    process =  subprocess.run(command, 
                         stdout=subprocess.PIPE, 
                         universal_newlines=True, 
                         shell=True)

