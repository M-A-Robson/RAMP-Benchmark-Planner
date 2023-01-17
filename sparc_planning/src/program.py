
from dataclasses import dataclass
from asp import *

@dataclass
class AnswerSetProgram:
    
    def __init__(self, nsteps:int,
                 agents:List[Agent],
                 action_set:List[ActionDefinition],
                 domain_sorts:List[Sort],
                 inertial_nk_fluents: List[Fluent],
                 inertial_k_fluents: List[Fluent],
                 defined_fluents: List[Fluent],
                 sort_relations: List[Predicate],
                 ):
        # create default sorts
        actions = FluentSort('action', action_set)
        numSteps = Constant('numSteps', nsteps)
        boolean = BaseSort('boolean', ['true','false'])
        outcome = BaseSort('outcome', ['true','false','undet'])
        step = Range('step', 0, numSteps)      
        inertial_nk_fluent = FluentSort('inertial_nk_fluents', inertial_nk_fluents)
         # add test actions by agent
        for agent in agents:
            actions.add_item(ActionDefinition('test',agent,inertial_nk_fluent))
            inertial_k_fluents += [Fluent('observed',[agent,inertial_nk_fluent]),
                                  Fluent('dir_obs',[agent, inertial_nk_fluent,outcome]),
                                  Fluent('indir_obs',[agent,inertial_nk_fluent,outcome])]
            defined_fluents += [Fluent('may_dscvr',[agent,inertial_nk_fluent]),
                                Fluent('can_test',[agent,inertial_nk_fluent])]
        inertial_k_fluent = FluentSort('inertial_k_fluents', inertial_k_fluents)
        inertial_fluent = GroupSort('inertial_fluent', [inertial_nk_fluent, inertial_k_fluent])
        defined_fluent = FluentSort('defined_fluent', defined_fluents)
        fluent = GroupSort('fluent', [inertial_fluent, defined_fluent])
        
        
        self.constants = [numSteps]
        #sorts = agents + objects + locations
        #        + named ranges + actions
        #        + fluents + statics 
        #        + constants
        self.sorts = [  boolean, outcome, step, *domain_sorts, *agents, actions, inertial_k_fluent,
                        inertial_nk_fluent, inertial_fluent, defined_fluent, fluent
                     ]
        #predicates = sort relationships
        self.predicates = [*sort_relations,
                           Predicate('val',[fluent,boolean,step]),
                           Predicate('is_defined',[fluent]),
                           Predicate('occurs',[actions, step]),
                           Predicate('hpd', [actions, step]),
                           Predicate('success', []),
                           Predicate('goal', [step]),
                           Predicate('something_happened', [step]),
                          ]
        #rules = causal laws + state constraints
        #        + executability conditions
        self.rules = []
        # predicates to display in result
        self.display = ['occurs.']
        # starting state (grounded predicates)
        self.start_state = []
        # history of actions and observations
        self.history = []
        # goal for agent
        self.goal = None
        # tick up steps
        self.current_step = 0
        
    def to_sparc(self) -> str:
        const = [c.to_sparc() for c in self.constants]
        sort_header = '\nsorts\n'
        sorts = [s.to_sparc() for s in self.sorts]
        predicate_header = '\npredicates\n'
        preds = [p.to_sparc() for p in self.predicates]
        rules_header = '\nrules\n'
        rules = [r.to_sparc() for r in self.rules]
        history = self.history
        state = [i.to_sparc() for i in self.start_state]
        disp_header = '\ndisplay\n'
        disp = self.display
        return '\n'.join([*const,sort_header,*sorts,predicate_header,
                          *preds,rules_header,*rules,*history,*state,disp_header,*disp])
    
    def add_observation_to_history(self,agent:Agent,fluent:Fluent,value:Any,boolean:bool,timestep:int):
        #todo check that fluent is observable and value is in range(f)
        self.history.append(f'val(dir_obs({agent.name},{fluent.name},{value}),{str(boolean).lower()},{timestep}).')

    def add_action_to_history(self,action:ActionInstance):
        #todo check that action is in action set
        self.history.append(f'hpd({action.action.name}({action.agent},{action.target}),{action.step}).')
        self.current_step += 1
        
        
def main():
    
    place_c = BaseSort("place_c", ['office','library','kitchen'])
    robot = Agent()
    
    tb = BaseSort('textbook', ['text0'])
    o = GroupSort('object', [tb])
    things = GroupSort('thing', [robot, o])
    
    move = ActionDefinition('move', robot, place_c)
    grasp = ActionDefinition('grasp', robot, o)
    putdown = ActionDefinition('putdown', robot, o)
    
    in_hand = Fluent('in_hand',[robot,o])
    loc_c = Fluent('loc_c',[things,place_c])
    
    next_to_c = Predicate('next_to_c',[place_c,place_c])
      
    asp = AnswerSetProgram(3,[robot],[move,grasp, putdown],[tb,o,things],[in_hand,loc_c],[],[],[next_to_c])
    #TODO make inertial knowledge fluents internal (observation driven)
    #TODO add default defined fluents (may_dscvr and can_test)
    print(asp.to_sparc())
    
if __name__ == "__main__":
    main()
