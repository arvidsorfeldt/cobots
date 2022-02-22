from typing import Tuple, List
import pytest
from predicates.state import State
from predicates import guards, actions
from predicates.guards import AlwaysTrue, AlwaysFalse
from model.model import the_model, Model
from model.operation import Operation, Transition
from planner.plan import plan

# ---------------------------------------------------------------------------
# ...
# ---------------------------------------------------------------------------

g = guards.from_str
a = actions.from_str


def test_simple_planner_1():
    """
    This test checks the implementation of the planner with a simple model
    """
    initial_state = State(
        v1 = False,
        v2 = 0
    )

    o1 = Operation(
        name=f"o1", 
        # enabled when v1 is false
        precondition=Transition("pre", g("!v1"), ()), 
        # the guard of the postcondition is only used when running the operation, not when planning
        postcondition=Transition("post", AlwaysTrue, a(f"v1")), 
        # the effects are only used when planning to simulate changes of sensors
        effects=(),
    )
    o2 = Operation(
        name=f"o2", 
        precondition=Transition("pre", g("v1 && v2 == 0"), ()),
        postcondition=Transition("post", AlwaysTrue, a(f"v2 += 1")),
        effects=(),
    )
    o3 = Operation(
        name=f"o3", 
        precondition=Transition("pre", g("v1 && v2 == 0"), ()),
        postcondition=Transition("post", AlwaysTrue, a(f"v2 += 2")),
        effects=(),
    )
    o4 = Operation(
        name=f"o4", 
        precondition=Transition("pre", g("v1 && v2 == 2"), ()),
        postcondition=Transition("post", AlwaysTrue, a(f"v2 += 1")),
        effects=(),
    )
    o5 = Operation(
        name=f"o5", 
        precondition=Transition("pre", g("v1"), ()),
        postcondition=Transition("post", AlwaysTrue, a(f"v2 <- 0")),
        effects=(),
    )
    simple_model = Model(initial_state, {
        o1.name: o1,
        o2.name: o2,
        o3.name: o3,
        o4.name: o4,
        o5.name: o5,
    })

    goal = g("v2 == 3")
    p = plan(initial_state, goal, simple_model)
    assert p != None
    assert len(p) != 0
    assert p == [o1.name, o3.name, o4.name]

    goal = g("v2 == 1")
    p = plan(initial_state, goal, simple_model)
    assert p == [o1.name, o2.name]

def test_simple_planner_2():
    """
    This test checks the implementation of the planner with a simple model
    """
    initial_state = State(
        v1 = False,
        v2 = 0
    )
    ops = {}
    for i in range(100):
        ops[f"o{i}"] = Operation(
            name=f"o{i}", 
            # enabled when v1 is false
            precondition=Transition("pre", g("!v1"), ()), 
            # the guard of the postcondition is only used when running the operation, not when planning
            postcondition=Transition("post", AlwaysTrue, a(f"v1")), 
            # the effects are only used when planning to simulate changes of sensors
            effects=(),
        )
    


    ops["final"] = Operation(
        name=f"final", 
        precondition=Transition("pre", g("v1 && v2 == 0"), ()),
        postcondition=Transition("post", AlwaysTrue, a(f"v2 += 1")),
        effects=(),
    )
    model = Model(initial_state, ops)

    goal = g("v2 == 1")
    p = plan(initial_state, goal, model)
    print(p)
    assert p != None
    assert len(p) == 2
    assert p[1] == "final"

def test_simple_planner_3():
    """
    This test checks the implementation of the planner with a simple model
    """
    initial_state = State(
        v1 = False,
        v2 = 0
    )
    ops = {}
    for i in range(100):
        ops[f"o{i}"] = Operation(
            name=f"o{i}", 
            # enabled when v1 is false
            precondition=Transition("pre", g(f"v2 == {i}"), ()), 
            # the guard of the postcondition is only used when running the operation, not when planning
            postcondition=Transition("post", AlwaysTrue, a(f"v2 +=1")), 
            # the effects are only used when planning to simulate changes of sensors
            effects=(),
        )

    model = Model(initial_state, ops)

    goal = g("v2 == 100")
    p = plan(initial_state, goal, model, 120)
    print(p)
    assert p != None
    assert len(p) == 100



# Use this test when you are working with the model 
def test_planner_real_model_1():
    """This method creates the test the planner that you will use for just a simple case"""
    m = the_model()
    
    goal = g("in_pos1 == empty")
    assert plan(m.initial_state, goal, m) == []
    
    
    goal = g("in_pos1 != empty")
    p = plan(m.initial_state, goal, m)
    print(f"plan: {p}")
    assert p == ['add_cube', 'to_input', 'pick_at_input', 'to_pos1', 'place_at_pos1']
    
    goal = g("in_pos1 != empty && in_pos2 != empty && in_pos3 != empty")
    p = plan(m.initial_state, goal, m)
    print(f"plan long: {p}")
    assert p != None
    assert len(p) == 15
    
    # here you should create more tests to check your model ...



