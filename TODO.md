# Outstanding Issues

## General TODOs

- [x] add usage instructions to README

## Coarse planning

- [x] Need to add pushing in coarse planner level

  - added push action
  - added misaligned fluent and casual law to assembly action

- [ ] Assembly action should be more understandable, rather than assemble(rob0, beam4), it should be assemble(rob0, beam4, beam5) to illustrate that beam4 is assembled into beam5. However, issues occur on capping where several beams are assembled together in one action.

  - seperate out with branch

- [x] Add heuristic to put in pins when beams have been assembled together (reduce branching in plans and match to benchmark scoring) similar to the take good actions example from literature.

  - added can_fasten fluent to track beams assembled but not yet fastened
  - added executability condition on assemble so that cannot assemble until beams fastened
  - added causal laws on assemble and fasten to undate can_fasten fluent

## Fine planning

- [ ] bug choosing wrong assembly actions e.g. choosing assemble_cap when should be assemble_square, likely caused by missing low level information in zooming. state constraints for fits_into may need rules for \_f versions

  - added new constraints need to test behaviours

- [x] performing assembly actions on links, limit to joints.

- [x] update with new features inherited from coarse level changes.
- [ ] test changes

- [x] add heuristic to grasp near to beam centre
