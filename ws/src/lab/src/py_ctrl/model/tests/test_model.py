import pytest
from predicates.state import State
from predicates import guards, actions
from model.operation import Transition, Operation
from predicates.errors import NextException
from model.model import Model, the_model

def test_model_creation():
    m = the_model()
    enabled_in_initial = [o for name, o in m.operations.items() if o.eval(m.initial_state)]
    assert len(enabled_in_initial) > 0


# write your own tests so that you know that the model you have created is the one you expected.
# for example, write a test for each operation so that it is enabled in the correct states and
# that it changes the state both when using next_planning and start and complete





