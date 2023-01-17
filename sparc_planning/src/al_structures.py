from typing import Dict, List, Optional, Union
from dataclasses import dataclass
from enum import Enum, auto
from abc import ABC

from sparc_io import SparcProg

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
            return f"#{self.name} = {{{','.join(self.instances)}}}."
        elif self.sort_type == SortType.SET:
            return f"#{self.name} = {' + '.join(self.instances)}."

class BasicSort(Sort):
    def __init__(self, name:str, instances:list[str]):
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


@dataclass
class Relation(Enum):
    GREATER_THAN = auto()
    GREATER_OR_EQUAL = auto()
    LESS_THAN = auto()
    LESS_OR_EQUAL = auto()
    EQUAL = auto()
    NOT_EQUAL = auto()

@dataclass
class Property:
    object1:str
    object2:str
    relation:Relation
    def to_string(self):
        rel = {
        Relation.GREATER_THAN : '>',
        Relation.GREATER_OR_EQUAL : '>=',
        Relation.LESS_THAN : '<',
        Relation.LESS_OR_EQUAL : '<=',
        Relation.EQUAL : '=',
        Relation.NOT_EQUAL : '!=',
        }
        return self.object1 + rel[self.relation] + self.object2

@dataclass
class ActionDefinition:
    name:str
    sorts:List[Sort]
    
    def to_sparc(self):
        return f"{self.name}({','.join([f'#{s.name}' for s in self.sorts])})"

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
        condition_strings = []
        for i in range(len(self.conditions)):
            s = ''
            if self.condition_values[i] == False:
                s += '¬'
            cond = self.conditions[i]
            if isinstance(cond, Property):
                s += cond.to_string()
            else:
                s += f"{cond.name}({','.join(self.condition_object_instance_names[i])})"
            condition_strings.append(s)
        fluent_val = ''
        if self.fluent_value == False: fluent_val = '¬'
        ret = f"{self.action.name}({','.join(self.action_object_instance_names)}) " + \
            "causes " +  f"{fluent_val}{self.fluent_affected.name}({','.join(self.fluent_object_instance_names)}) " + \
            "if " + ', '.join(condition_strings)
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
                s += f"holds({cond.name}({','.join(self.condition_object_instance_names[i])}, {v}, I)"
            else:
                if self.condition_values[i] == False:
                    s += 'not'
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
    head:Optional[Func] = None
    head_object_instance_names: Optional[List[str]] = None
    conditions: Optional[List[Union[Func,Property]]] = None
    condition_object_instance_names: Optional[List[List[str]]] = None
    condition_values: Optional[List[bool]] = None
    head_value: bool = True
    
    def to_AL(self) -> str:
        """returns AL string of form 'l if p0,..pm' """
        head_val = ''
        if self.head_value == False: head_val = '¬'
        head = f"{head_val}{self.head.name}"
        if self.head_object_instance_names:
            head += f"({', '.join(self.head_object_instance_names)})"
        if self.conditions:
            condition_strings = []
            for i in range(len(self.conditions)):
                s = ''
                if self.condition_values[i] == False:
                    s += '¬'
                cond = self.conditions[i]
                if isinstance(cond, Property):
                    s += cond.to_string()
                else:
                    s += f"{cond.name}({','.join(self.condition_object_instance_names[i])})"
                condition_strings.append(s)
            return head + "if " + ', '.join(condition_strings)
        return head

    def to_sparc(self) -> str:
        """returns SPARC Rule string of form 'l :- p0,..pm.' """
        # form main causal relationship 'holds(l_in, value, I+1) :- occurs(a,I).
        if self.head.func_type == FuncType.FLUENT:
            head = f"holds({self.head.name}({','.join(self.fluent_object_instance_names)}), {str(self.head_value).lower()}, I) "
        else: 
            head = ''
            if self.head_value == False: head = '-'
            head += f"{self.head.name}"
            if self.head_object_instance_names:
                head += f"({','.join(self.fluent_object_instance_names)})"
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
                    s += 'not'
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
                s += cond.to_string()
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
                    s += 'not'
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
    fluent:Func
    object_instances:List[str]
    value:bool

    def to_sparc(self) -> str:
        return f"holds({self.fluent.name}({','.join(self.object_instances)}), {str(self.value).lower()}, I)"

@dataclass
class Action:
    """simple class for grouping actions with their causal laws and executability conditions"""
    action_def:ActionDefinition
    causal_laws:List[CausalLaw]
    executability_conditions:List[ExecutabilityCondition]
    
    
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
    
    def to_sparc_program(self) -> SparcProg:
        # create constants
        sparc_constants = [f'#const numSteps = {self.planning_steps}.']
        if self.constants:
            sparc_constants += [c.to_sparc() for c in self.constants]
        
        # create sorts
        sparc_sorts = [s.to_sparc() for s in self.sorts]
        # add action sort
        sparc_sorts.append(f"#action = {', '.join([a.action_def.to_sparc() for a in self.actions])}.")
        # handle case of empty set for defined fluents
        if self.defined_fluents:
            defined_fluent_sort = f"#defined_fluent = {'+ '.join([d.to_sparc()[:-1] for d in self.defined_fluents])}."
        else: defined_fluent_sort = r'#defined_fluent = {}.'
        # create special sorts for inertial fluents, fluent, boolean, step, and outcome
        sparc_sorts += ['#boolean = {true, false}.', '#outcome = {true, false, undet}.',
                        defined_fluent_sort,
                        f"#inertial_fluent = {'+ '.join([i.to_sparc()[:-1] for i in self.inertial_fluents])}.",
                        '#fluent = #inertial_fluent + #defined_fluent.',
                        f'#step = 0..numSteps.'
                        ]

        # create predicates
        # add statics
        sparc_predicates = []
        if self.statics:
            sparc_predicates += [stat.to_sparc() for stat in self.statics]
        # add planning predicates
        sparc_predicates += ['holds(#fluent, #boolean, #step).',
                              'occurs(#action, #step).',
                              'success().','goal(#step).',
                              'something_happened(#step).']
        
        # create rules
        rules = []
        # state constraints
        if self.state_constraints:
            rules += [s.to_sparc for s in self.state_constraints]
        # causal laws and executability conditions
        for a in self.actions:
            rules += [c.to_sparc() for c in a.causal_laws]
            rules += [e.to_sparc() for e in a.executability_conditions]
        # planning laws
        rules += [ 
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
        goal = 'goal(I) :- '
        goal_items = [item.to_sparc() for item in self.goal_description]
        goal += f"{' + '.join(goal_items)}."
        rules.append(goal)

        # domain setup
        rules += self.domain_setup
        return SparcProg(sparc_constants, sparc_sorts, sparc_predicates, rules, self.display_hints)

def test():
    robot = BasicSort('robot', ['rob0'])
    thing = BasicSort('thing', ['textbook', 'pickle'])
    put_down = ActionDefinition('putdown',[robot, thing])
    in_hand = Func('in_hand',[robot,thing],FuncType.FLUENT)

    pd_ec1 = ExecutabilityCondition(
                        action = put_down,
                        object_instances= {'R':robot, 'T':thing},
                        action_object_instance_names=['R','T'],
                        conditions= [in_hand],
                        condition_object_instance_names= [['R','T']],
                        condition_values= [False] )

    pd_c = CausalLaw(put_down,
                in_hand,
                False,
                {'R':robot,'T':thing},
                ['R','T'],['R','T'],
                )

    A = Action(put_down,[pd_c],[pd_ec1])

    ALD = ActionLangSysDesc(
        sorts=[robot, thing],
        inertial_fluents=[in_hand],
        actions=[A],
        domain_setup=['holds(in_hand(rob0,textbook), true, 0).'],
        goal_description=GoalDefinition(in_hand,['rob0','textbook'],False),
        display_hints=['occurs.'], planning_steps=1)

    ASP = ALD.to_sparc_program()
    ASP.save('test_sparc.sp')
    
if __name__ == "__main__":
    test()
        