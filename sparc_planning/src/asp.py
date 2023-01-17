from typing import Any, List, Optional, Union
from dataclasses import dataclass, field

@dataclass(frozen=True)
class Constant:
    """define constants, immutable"""
    name:str
    value:int
    
    def to_sparc(self) -> str:
        return f'#const {self.name} = {self.value}.'
    
    def __str__(self) -> str:
        return self.name
    

@dataclass
class Sort:
    name:str

@dataclass
class Range(Sort):
    """define a range with integers or defined constants"""
    start:Union[int, Constant]
    stop:Union[int, Constant]
    expansion:Optional[str]
    
    def to_sparc(self) -> str:
        if self.expansion:
            return  f'#{self.name} = [{self.expansion}][{str(self.start)}..{str(self.stop)}].'
        else:
            return f'#{self.name} = {str(self.start)}..{str(self.stop)}.'

@dataclass
class BaseSort(Sort):
    """sort definition for ground instances"""
    instances:List[str] = field(default_factory=lambda : [])
    
    def to_sparc(self) -> str:
        return f"#{self.name} = {{{','.join(self.instances)}}}."
    
@dataclass
class Agent(BaseSort):
    """special base sort for easier tracking of agents"""
    def __init__(self, agent_type_name='robot', agent_names:List[str]=['rob0']):
        super().__init__(agent_type_name,agent_names)

@dataclass
class GroupSort(Sort):
    """sort constructed from a set of subsorts"""
    subsorts:List[Sort] = field(default_factory=lambda : [])
    
    def to_sparc(self) -> str:
        return f"#{self.name} = {' + '.join([f'#{s.name}' for s in self.subsorts])}."

@dataclass
class Static:
    """define sort relationships"""
    name:str = ''
    sorts:List[Sort] = field(default_factory=lambda : [])
    
    def to_sparc(self) -> str:
        return f"{self.name}({','.join([f'#{s.name}' for s in self.sorts])})."

@dataclass
class Fluent:
    """defines functions who's value will change during execution"""
    name:str = ''
    sorts:List[Sort] = field(default_factory=lambda : [])
    
    def to_sparc(self) -> str:
        return f"{self.name}({','.join([f'#{s.name}' for s in self.sorts])})"
    
@dataclass
class FluentSort:
    """define sorts for groups of fluents"""
    name:str
    fluents:List[Fluent]
    
    def to_sparc(self) -> str:
        return f"#{self.name} = {' + '.join(f.to_sparc() for f in self.fluents)}."
    
    def add_item(self, fluent:Fluent) -> None:
        self.fluents.append(fluent)
    
    def remove_item(self, fluent:Fluent) -> None:
        self.fluents.remove(fluent)
    
@dataclass
class ActionDefinition(Fluent):
    """define possible actions, special type of fluent"""
    def __init__(self, name:str, agent:Agent, affected_sort:Sort):
        super().__init__(name,[agent,affected_sort])

@dataclass
class ActionInstance:
    '''Record of action taken'''
    agent:str
    action:ActionDefinition
    target:str
    step:int
    
    def post_init(self):
        """Checks that action has valid agent/target combination"""
        action_agent_list, action_target_list = self.action.fluents
        if self.agent not in action_agent_list:
            return ValueError, f'{self.agent} not compatible with action {self.action.name}\nexpected one of {action_agent_list}'
        if self.target not in action_target_list:
            return ValueError, f'{self.target} not compatible with action {self.action.name}\nexpected one of {action_target_list}'
            
        
        
def main():
    c = Constant('numSteps', 2)
    place_c = BaseSort("place_c", ['office','library','kitchen'])
    """
    place_f = BaseSort("place_f", ["c1","c2","c3","c4","c5","c6"])
    print(place_f.to_sparc())"""

    steps = Range('step',0,c)
    robot = Agent()
    tb = BaseSort('textbook', ['text0'])
    o = GroupSort('object', [tb])
    things = GroupSort('thing', [robot, o])
    move = ActionDefinition('move', robot, place_c)
    grasp = ActionDefinition('grasp', robot, o)
    actions = FluentSort('action', [move])
    actions.add_item(grasp)
    
if __name__ == "__main__":
    main()





