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
from typing import Dict, List, Optional, Tuple, Union
from dataclasses import dataclass
from enum import Enum, auto
from sparc_planning.src.sparc_io import SparcProg
import logging
import itertools

logger = logging.getLogger()

@dataclass(frozen=True)
class Constant:
    """define constants, immutable"""
    name:str
    value:int
    
    def __init__(self, name:str, value:int):
        if name == 'numSteps':
            raise ValueError('numSteps is a reserved constant name')
        self.name = name
        self.value = value
    
    def to_sparc(self) -> str:
        return f'#const {self.name} = {self.value}.'
    
    def to_AL(self) -> str:
        return f'constant: {self.name}.'
    
    def __str__(self) -> str:
        return self.name
    
class SortType(Enum):
    BASIC = auto()
    SET = auto()

class Sort:
    def __init__(self, name:str, instances:List[str], sort_type:SortType):
        self.name = name
        self.instances = instances
        self.sort_type = sort_type
        
    def to_sparc(self) -> str:
        if self.sort_type == SortType.BASIC:
            if len(self.instances) == 0:
                logging.warning(f'Adding dummy object to empty sort: {self.name}')
                return f"#{self.name} = {{dummy_{self.name}}}."
            return f"#{self.name} = {{{','.join(self.instances)}}}."
        elif self.sort_type == SortType.SET:
            return f"#{self.name} = {' + '.join(self.instances)}."
        
    def to_AL(self) -> str:
        if self.sort_type == SortType.BASIC:
            return f"{self.name}"
        elif self.sort_type == SortType.SET:
            return f"{self.name}, with subsorts: {' , '.join([i[1:] for i in self.instances])}."
        
    def remove_instance_by_name(self, name:str) -> None:
        try:
            self.instances.remove(name)
        except ValueError:
            logger.warning(f'Instance: {name} not found in sort {self.name}.')


class BasicSort(Sort):
    def __init__(self, name:str, instances:List[str]):
        super().__init__(name, instances, SortType.BASIC)

class RangeSort(Sort):
    def __init__(self, name:str, start:Union[int,Constant], stop:Union[int,Constant],
        expansion:str = ''):
        if isinstance(start, Constant): start = start.value
        if isinstance(stop, Constant): stop = stop.value
        instances = [f'{expansion}{str(i)}' for i in range(start, stop)]
        super().__init__(name, instances, SortType.BASIC)

class SuperSort(Sort):
    def __init__(self, name:str, subsorts:List[Sort]):
        self.subsorts = subsorts
        instances = [f'#{s.name}' for s in subsorts]
        super().__init__(name, instances, SortType.SET)

class FuncType(Enum):
    FLUENT = auto()
    STATIC = auto()
    OTHER = auto()

@dataclass
class Func:
    """base class for static, fluent, and predicate functions"""
    name:str
    sorts:Optional[List[Sort]]
    func_type:FuncType
    
    def to_sparc(self) -> str:
        if self.sorts:
            return f"{self.name}({', '.join([f'#{s.name}' for s in self.sorts])})."
        return f'{self.name}().'
    
    def to_AL(self) -> str:
        if self.sorts:
            return f"{self.name}({', '.join([f'{s.name}' for s in self.sorts])})."
        return f'{self.name}().'


@dataclass
class Relation(Enum):
    GREATER_THAN = auto()
    GREATER_OR_EQUAL = auto()
    LESS_THAN = auto()
    LESS_OR_EQUAL = auto()
    EQUAL = auto()
    NOT_EQUAL = auto()
    IS_OF_SORT = auto()

@dataclass
class Property:
    object1:str
    object2:str
    relation:Relation

    def to_string(self):
        # a bit clunky but works for now
        if self.relation.value == Relation.IS_OF_SORT.value:
            return f'#{self.object1}({self.object2})'
        rel = {
        Relation.GREATER_THAN.value : '>',
        Relation.GREATER_OR_EQUAL.value : '>=',
        Relation.LESS_THAN.value : '<',
        Relation.LESS_OR_EQUAL.value : '<=',
        Relation.EQUAL.value : '=',
        Relation.NOT_EQUAL.value : '!=',
        }
        return self.object1 + rel[self.relation.value] + self.object2

@dataclass
class ActionDefinition:
    name:str
    sorts:List[Sort]
    
    def to_sparc(self):
        return f"{self.name}({','.join([f'#{s.name}' for s in self.sorts])})"
    
    def to_AL(self):
        return f"{self.name}({','.join([f'{s.name}' for s in self.sorts])})"

@dataclass
class CausalLaw:
    action:ActionDefinition
    fluent_affected:Func
    fluent_value:bool
    object_instances: Dict[str,Sort]
    action_object_instance_names: List[str]
    fluent_object_instance_names: List[str]
    conditions: Optional[List[Union[Func,Property,Sort]]] = None
    condition_object_instance_names: Optional[List[List[str]]] = None
    condition_values: Optional[List[bool]] = None
 
    def to_AL(self) -> str:
        """returns AL string of form 'a causes l_in if p0,..pm' """
        fluent_val = ''
        if self.fluent_value == False: fluent_val = '¬'
        ret = f"{self.action.name}({','.join(self.action_object_instance_names)}) " + \
            "causes " +  f"{fluent_val}{self.fluent_affected.name}({','.join(self.fluent_object_instance_names)}) "
        if not self.conditions:
            return ret
        condition_strings = []
        for i in range(len(self.conditions)):
            s = ''
            if self.condition_values[i] == False:
                s += '¬'
            cond = self.conditions[i]
            if isinstance(cond, Property):
                c = cond.to_string()
                if cond.relation.value == Relation.IS_OF_SORT.value:
                    c = c[1:]
                s += c
            else:
                s += f"{cond.name}({','.join(self.condition_object_instance_names[i])})"
            condition_strings.append(s)
        ret += "if "
        ret += ', '.join(condition_strings)
        return ret

    def to_sparc(self) -> str:
        """returns SPARC Rule string of form 'holds(l_in, I+1) :- occurs(a,I), p0,..pm.' """
        # form main causal relationship 'holds(l_in, value, I+1) :- occurs(a,I).
        ret = f"holds({self.fluent_affected.name}({','.join(self.fluent_object_instance_names)}), {str(self.fluent_value).lower()}, I+1) " + \
        f":- occurs({self.action.name}({','.join(self.action_object_instance_names)}), I)."
        if not self.conditions:
            return ret
        condition_strings = []
        for i in range(len(self.conditions)):
            s = ''
            cond = self.conditions[i]
            # fluents
            if isinstance(cond, Func) and cond.func_type == FuncType.FLUENT:
                v = 'true'
                if self.condition_values[i] == False:
                    v = 'false'
                s += f"holds({cond.name}({','.join(self.condition_object_instance_names[i])}), {v}, I)"
            else:
                if self.condition_values[i] == False:
                    s += 'not '
                if isinstance(cond, Sort):
                    s += '#'
                if isinstance(cond, Property):
                    s += cond.to_string()
                else:
                    # static or other
                    s += f"{cond.name}({','.join(self.condition_object_instance_names[i])})"
            condition_strings.append(s)
        # add conditionals
        return ret[:-1] + ', ' + ', '.join(condition_strings) + '.'

@dataclass 
class StateConstraint:
    object_instances: Dict[str,Sort]
    head:Optional[Func|str] = None
    head_object_instance_names: Optional[List[str]] = None
    conditions: Optional[List[Union[Func,Property]]] = None
    condition_object_instance_names: Optional[List[List[str]]] = None
    condition_values: Optional[List[bool]] = None
    head_value: bool = True
    
    def to_AL(self) -> str:
        """returns AL string of form 'l if p0,..pm' """
        head_val = ''
        head = ''
        if self.head:
            if self.head_value == False: head_val = '¬'
            if isinstance(self.head,Func):
                head = f"{head_val}{self.head.name}"
                if self.head_object_instance_names:
                    head += f"({', '.join(self.head_object_instance_names)})"
            else: head = head_val + self.head
        if self.conditions:
            condition_strings = []
            for i in range(len(self.conditions)):
                s = ''
                if self.condition_values[i] == False:
                    s += '¬'
                cond = self.conditions[i]
                if isinstance(cond, Property):
                    c = cond.to_string()
                    if cond.relation.value == Relation.IS_OF_SORT.value:
                        c = c[1:]
                    s += c
                else:
                    s += f"{cond.name}({','.join(self.condition_object_instance_names[i])})"
                condition_strings.append(s)
            return head + " if " + ', '.join(condition_strings)
        return head

    def to_sparc(self) -> str:
        """returns SPARC Rule string of form 'l :- p0,..pm.' """
        # form main causal relationship 'holds(l_in, value, I+1) :- occurs(a,I).
        head = ''
        if self.head:
            if isinstance(self.head,Func):
                if self.head.func_type == FuncType.FLUENT:
                    head = f"holds({self.head.name}({','.join(self.head_object_instance_names)}), {str(self.head_value).lower()}, I) "
                else: 
                    if self.head_value == False: head = '-'
                    head += f"{self.head.name}"
                    if self.head_object_instance_names:
                        head += f"({','.join(self.head_object_instance_names)})"
            else:
                if self.head_value == False: head = '-'
                head += self.head
        if not self.conditions:
            head += '.'
            return head
        condition_strings = []
        for i in range(len(self.conditions)):
            s = ''
            # fluents
            cond = self.conditions[i]
            if isinstance(cond, Func) and cond.func_type == FuncType.FLUENT:
                s += f"holds({cond.name}({','.join(self.condition_object_instance_names[i])}), {str(self.condition_values[i]).lower()}, I)"
            else:
                if self.condition_values[i] == False:
                    s += 'not '
                if isinstance(cond, Sort):
                    s += '#'
                if isinstance(cond, Property):
                    s += cond.to_string()
                else:
                    # static, sort instance, or other
                    s += f"{cond.name}({','.join(self.condition_object_instance_names[i])})"
            condition_strings.append(s)
        # add conditionals
        ret = head + ':- ' + ', '.join(condition_strings) + '.'
        return ret

@dataclass
class ExecutabilityCondition:
    """executability condition for action limits
    
    implementation detail: always put positive conditions before negetive
    where possible. Otherwise, in zooming conditions are checked in order
    so a #sort might eliminate the exec condition but a 'not #sort' might
    cause the action to be removed prematurely.
    """
    action:ActionDefinition
    object_instances: Dict[str,Sort]
    action_object_instance_names: List[str]
    conditions: List[Union[Func,Property]]
    condition_object_instance_names: List[List[str]]
    condition_values: List[bool]
    
    def to_AL(self) -> str:
        """returns AL string of form 'impossible a if p0,..pm' """
        condition_strings = []
        for i in range(len(self.conditions)):
            s = ''
            if self.condition_values[i] == False:
                s += '¬'
            cond = self.conditions[i]
            if isinstance(cond, Property):
                c = cond.to_string()
                if cond.relation.value == Relation.IS_OF_SORT.value:
                    c = c[1:]
                s += c
            else:
                s += f"{cond.name}({','.join(self.condition_object_instance_names[i])})"
            condition_strings.append(s)
        ret = f"impossible {self.action.name}({','.join(self.action_object_instance_names)}) " + \
            "if " + ', '.join(condition_strings)
        return ret

    def to_sparc(self) -> str:
        """returns SPARC Rule string of form '-occurs(a, I) :- p0,..pm.' """
        condition_strings = []
        for i in range(len(self.conditions)):
            s = ''
            # fluents
            cond = self.conditions[i]
            if isinstance(cond, Func) and cond.func_type == FuncType.FLUENT:
                if self.condition_values[i] == False: s += 'not '
                s += f"holds({cond.name}({','.join(self.condition_object_instance_names[i])}), true, I)"
            else:
                if self.condition_values[i] == False:
                    s += 'not '
                if isinstance(cond, Sort):
                    s += '#'
                if isinstance(cond, Property):
                    s += cond.to_string()
                else:
                    # static or other
                    s += f"{cond.name}({','.join(self.condition_object_instance_names[i])})"
            condition_strings.append(s)
        return f"-occurs({self.action.name}({','.join(self.action_object_instance_names)}), I) :- {', '.join(condition_strings)}."

@dataclass
class GoalDefinition:
    fluent_name:str
    object_instances:List[str]
    value:bool

    def to_sparc(self) -> str:
        return f"holds({self.fluent_name}({','.join(self.object_instances)}), {str(self.value).lower()}, I)"

@dataclass
class Action:
    """simple class for grouping actions with their causal laws and executability conditions"""
    action_def:ActionDefinition
    causal_laws:List[CausalLaw]
    executability_conditions:List[ExecutabilityCondition]

    @property
    def full_sort_signature(self):
        sort_sig = set()
        for cl in self.causal_laws:
            if not cl.conditions:continue
            for fun in cl.conditions:
                if not isinstance(fun, Property):
                    sort_sig = sort_sig.union({*[s.name for s in fun.sorts]})
        for ec in self.executability_conditions:
            if not ec.conditions:continue
            for fun in ec.conditions:
                if not isinstance(fun, Property):
                    sort_sig = sort_sig.union({*[s.name for s in fun.sorts]})
        return sort_sig
    
@dataclass
class ActionInstance:
    """simple class for recording actions which trigger state transitions
    args:
        action (Action) - type of action taken.
        object_constants (List[str]) - ground instances involved eg. ['rob0','textbook0'].
        timestep - plan step time at which action occured.    
    """
    action:Action
    object_constants:List[str]
    timestep:int
    
@dataclass
class ActionLangSysDesc:
    sorts : List[Sort]
    inertial_fluents : List[Func]
    actions : List[Action]
    domain_setup : List[str]
    goal_description : List[GoalDefinition]
    defined_fluents : Optional[List[Func]] = None
    statics : Optional[List[Func]] = None
    state_constraints : Optional[List[StateConstraint]] = None
    constants : Optional[List[Constant]] = None
    display_hints: Optional[List[str]] = None
    planning_steps: int = 3
    start_step: int = 0

    def add(self,prog:ActionLangSysDesc) -> None:
        """combine another ALSD into this object"""
        # add sorts
        for sort in prog.sorts:
            added = False
            for s in self.sorts:
                # new prog has sort of same name as existing sort
                if s.name == sort.name:
                    if s.sort_type != sort.sort_type:
                        raise ValueError(f'new sort {sort.name} is of type {sort.sort_type}, but this sort already exisits with type {s.sort_type}')
                    s.instances = [*{*s.instances}.union({*sort.instances})] # update instances
                    added = True
                if not added:
                    # add new sort
                    self.sorts.append(sort)

        #inertial fluents
        i_names = [inertial_fluent.name for inertial_fluent in self.inertial_fluents]
        for iflu in prog.inertial_fluents:
            if iflu.name in i_names:
                logger.warning("Ignoring fluent '{iflu.name}' --> fluent with this name is already defined")
            else:
                self.inertial_fluents.append(iflu)

        #defined fluents are optional
        if not self.defined_fluents:
            self.defined_fluents = prog.defined_fluents
        else:
            d_names = [d_fluent.name for d_fluent in self.d_fluents]
            for dflu in prog.defined_fluents:
                if dflu.name in d_names:
                    logger.warning("Ignoring fluent '{dflu.name}' --> fluent with this name is already defined")
                else:
                    self.defined_fluents.append(dflu)

        #constants are optional
        if not self.constants:
            self.constants = prog.constants
        else:
            c_names = [c.name for c in self.constants]
            for cons in prog.constants:
                if cons.name in c_names:
                    logger.warning("Ignoring constant '{cons.name}' --> constant with this name is already defined")
                else:
                    self.constants.append(cons)

        #statics are optional
        if not self.statics:
            self.statics = prog.statics
        else:
            s_names = [s.name for s in self.statics]
            for stat in prog.statics:
                if stat.name in s_names:
                    logger.warning("Ignoring static '{stat.name}' --> static with this name is already defined")
                else:
                    self.statics.append(stat)
        
        # display_hints: Optional[List[str]] = None
        self.display_hints = [*{*self.display_hints}.union({*prog.display_hints})]

        # actions : List[Action]
        for act in prog.actions:
            a_names = [a.action_def.name for a in self.actions]
            if act.action_def.name in a_names:
                logger.warning("Ignoring action '{act.action_def.name}' --> action with this name is already defined")
                continue
            self.actions.append(act)

        # domain_setup : List[str]
        self.domain_setup = [*{*self.domain_setup}.union({*prog.domain_setup})]
        
        # goal_description : List[GoalDefinition]
        goals_as_str = [g.to_sparc() for g in self.goal_description]
        for goal in prog.goal_description:
            if goal.to_sparc() in goals_as_str:
                continue
            self.goal_description.append(goal)

        # state_constraints : Optional[List[StateConstraint]] = None
        if not self.state_constraints:
            self.state_constraints = prog.state_constraints
        else:
            const_as_str = [c.to_sparc() for c in self.state_constraints]
            for p in prog.state_constraints:
                if p.to_sparc in const_as_str:
                    continue
                self.state_constraints.append(p)      
    
    def complete_domain_setup_fluent(self,inertial_fluent:Func,value:bool,exceptions:Optional[List[Tuple[str]]]=None)->None:
        """
        Edits domain_setup to indicate that for all valid objects this fluent has a specific value.
        Exceptions must be in correct order to match fluent sort arrangement.
        Does not apply closed world assumption on exceptions.
        e.g. if fluent 'in_hand' has sorts: robot and thing:  

        complete_domain_setup_fluent(in_hand,False,[(rob0,textbook),(rob1,cup)])
        
        would indicate that holds(in_hand(rob0,{things}-textbook),false,0). similarly
        holds(in_hand(rob1,{things}-cup), false, 0).

        limited to fluents with 2 or less sorts.

        Args:
            inertial_fluent (Func): fluent
            value (bool): true/false
            exception (Optional[List[Tuple[str]]]): any object names to skip
                
        raises:
            ValueError: if fluent not defined
            AssertionError: if fluent has too many sorts (combinatorial explosion)
        """
        if not inertial_fluent in self.inertial_fluents:
            raise ValueError(f'fluent {inertial_fluent.name} not in inertial_fluents')
        instances = []
        for sort in inertial_fluent.sorts:
            instances.append(self.get_sort_objects(sort))
        assert len(instances) < 3, f'Fluent {inertial_fluent.name} has too many sorts'
        if len(instances) == 1: 
            self.domain_setup.extend([f"holds({inertial_fluent.name}({inst}),{str(value).lower()},0)." for inst in instances[0]])
            return    
        combinations = instances[0]
        for i in range(1,len(instances)):
            combinations = list(zip(combinations,element) for element in itertools.product(instances[i],repeat=len(combinations)))
        for combination in combinations:
            if exceptions and (combination in exceptions):
                self.domain_setup.append(f"holds({inertial_fluent.name}({','.join(list(list(combination)[0]))}),{str(not value).lower()},0).")
            else:
                self.domain_setup.append(f"holds({inertial_fluent.name}({','.join(list(list(combination)[0]))}),{str(value).lower()},0).")

    def get_sort_objects(self,sort):
        sort_dict = dict(zip([s.name for s in self.sorts],self.sorts))
        objects = []
        if isinstance(sort, SuperSort):
            for instance in sort.instances:
                objects += self.get_sort_objects(sort_dict[instance[1:]])
        else:
            objects += sort.instances
        return objects

    def to_sparc_program(self) -> SparcProg:
        # create constants
        logging.debug('Parsing constants...')
        sparc_constants = [f'#const numSteps = {self.planning_steps}.', f'#const startStep = {self.start_step}.']
        if self.constants:
            sparc_constants += [c.to_sparc() for c in self.constants]
        
        # create sorts
        logging.debug('Parsing sorts...')
        sparc_sorts = [s.to_sparc() for s in self.sorts]
        # add action sort
        sparc_sorts.append(f"#action = {' + '.join([a.action_def.to_sparc() for a in self.actions])}.")
        # create special sorts for inertial fluents, fluent, boolean, step, and outcome
        sparc_sorts += ['#boolean = {true, false}.', '#outcome = {true, false, undet}.',
                        f"#inertial_fluent = {'+ '.join([i.to_sparc()[:-1] for i in self.inertial_fluents])}.",
                        f'#step = startStep..numSteps.'
                        ]
        # handle case of empty set for defined fluents
        if self.defined_fluents:
            sparc_sorts.extend([f"#defined_fluent = {'+ '.join([d.to_sparc()[:-1] for d in self.defined_fluents])}.",
                            '#fluent = #inertial_fluent + #defined_fluent.'])
        else: 
            sparc_sorts.append('#fluent = #inertial_fluent.')
        

        # create predicates
        # add statics
        logging.debug('Parsing predicates...')
        sparc_predicates = []
        if self.statics:
            sparc_predicates += [stat.to_sparc() for stat in self.statics]
        # add planning predicates
        sparc_predicates += ['holds(#fluent, #boolean, #step).',
                              'occurs(#action, #step).',
                              'success().','goal(#step).',
                              'something_happened(#step).',
                              ]
        
        # create rules
        logging.debug('Parsing rules...')
        rules = []
        # state constraints
        if self.state_constraints:
            for s in self.state_constraints:
                try:
                    rules.append(s.to_sparc())
                except Exception as e:
                    logging.error(f'Error parsing state_constraint: {s}')
                    logging.error(e)
        # causal laws and executability conditions
        for a in self.actions:
            logging.debug(f'Parsing action: {a.action_def.name}...')
            rules.append('')
            rules += [c.to_sparc() for c in a.causal_laws]
            rules += [e.to_sparc() for e in a.executability_conditions]
        # planning laws
        logging.debug('Adding planning rules...')
        rules += [ 
                '',
                r'% planning rules',
                '-holds(F, V2, I) :- holds(F, V1, I), V1!=V2.',#Fluents can only have one value at a time...
                'holds(F, Y, I+1) :- #inertial_fluent(F), holds(F, Y, I), not -holds(F, Y, I+1), I < numSteps.', #inertia
                '-occurs(A,I) :- not occurs(A,I).', # CWA for Actions
                'success :- goal(I), I <= numSteps.', # define success
                ':- not success.', # Failure is not an option
                'occurs(A, I) | -occurs(A, I) :- not goal(I).', #Cannot stop executing actions, until goal achieved
                '-occurs(A2, I) :- occurs(A1, I), A1 != A2.', #Cannot have two actions happening concurrently
                'something_happened(I) :- occurs(A, I).', #record what has happened
                ':- not goal(I), not something_happened(I).' #if goal isnt reached do something
                ]
        
        # goal
        logging.debug('Parsing goal...')
        goal = 'goal(I) :- '
        goal_items = [item.to_sparc() for item in self.goal_description]
        goal += f"{' , '.join(goal_items)}."
        rules.append('')
        rules.append(r'% goal definition')
        rules.append(goal)

        # domain setup
        logging.debug('Parsing domain setup...')
        rules.append('')
        rules.append(r'% domain setup')
        rules += self.domain_setup
        return SparcProg(sparc_constants, sparc_sorts, sparc_predicates, rules, self.display_hints)

    def to_action_language_description(self) -> List[str]:
        # create constants
        logging.debug('Parsing constants...')
        _constants = []
        if self.constants:
            _constants = [c.to_AL() for c in self.constants]
        
        # create sorts
        logging.debug('Parsing sorts...')
        _sorts = [s.to_AL() for s in self.sorts]
        
        # define actions
        _actions = [a.action_def.to_AL() for a in self.actions]

        # create special sorts for inertial fluents, fluent, boolean, step, and outcome
        _inertial_fluents = [i.to_AL() for i in self.inertial_fluents]

        # handle case of empty set for defined fluents
        _defined_fluents = []
        if self.defined_fluents:
            _defined_fluents = [d.to_AL() for d in self.defined_fluents]

        # create predicates
        # add statics
        logging.debug('Parsing predicates...')
        _statics = []
        if self.statics:
            _statics = [stat.to_AL() for stat in self.statics]
        
        # create rules
        logging.debug('Parsing rules...')
        _state_constraints = []
        # state constraints
        if self.state_constraints:
            for s in self.state_constraints:
                try:
                    _state_constraints.append(s.to_AL())
                except Exception as e:
                    logging.error(f'Error parsing state_constraint: {s}')
                    logging.error(e)

        # causal laws and executability conditions
        _action_impl = []
        for a in self.actions:
            logging.debug(f'Parsing action: {a.action_def.name}...')
            _action_impl.append('')
            _action_impl += [c.to_AL() for c in a.causal_laws]
            _action_impl += [e.to_AL() for e in a.executability_conditions]

        al_def = ['constants\n'] + _constants \
                + ['\nsorts\n'] + _sorts \
                + ['\nactions\n'] + _actions \
                + ['\nintertial fluents\n'] + _inertial_fluents \
                + ['\ndefined fluents\n'] + _defined_fluents \
                + ['\nstatics\n'] + _statics \
                + ['\nstate constraints\n'] + _state_constraints \
                + ['\naction causal laws and executability conditions'] + _action_impl
                
        return al_def

    def save_AL(self, save_location:str) -> None:
        al = self.to_action_language_description()
        header = [
            r'%%%%%%%%%%%%%%%%%%%%%%%%%%%%%','\n',
            r'%% AutoGenerated AL file','\n',
            r'%% Author: MARK ROBSON 2023','\n',
            r'%%%%%%%%%%%%%%%%%%%%%%%%%%%%%','\n','\n']
        with open(save_location, 'w') as f:
            f.writelines(header)
            f.writelines(line + '\n' for line in al)
        logging.info(f'Action language description written to {save_location}')

    def get_action_by_name(self, action_name:str) -> Action:
        for action in self.actions:
            if action.action_def.name == action_name:
                return action
        return ValueError(f'Action: {action_name}, not found in system description')
