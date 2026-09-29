# CheckStartStateCollision diagnostics use the current state instead of the request start state

When a supplied request start state is in collision, its diagnostic contacts should be computed from that same state.

The current scene state at x=0.8 is independently valid and the request start at x=0 is invalid, but the stage reports `0 contact(s) detected`. Passing the request start state to the contact query reports the expected `p3_start_box - slider` contact. This reproduced 3/3.
