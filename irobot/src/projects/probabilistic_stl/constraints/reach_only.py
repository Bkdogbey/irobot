from pdstl.operators import Eventually
from planning.environment import RectangularGoalPredicate


def build_reach_spec(env, T: int):
    """
    phi = Eventually[0, T](reach goal)

    Test 4 spec: point-to-point flight, no obstacles.
    Goal region defined in lab coordinates (meters from CF home).
    """
    preds = env.get_predicates()
    assert preds["goal"] is not None, "Environment must have a goal set"
    phi_reach = Eventually(preds["goal"], interval=[0, T])
    return phi_reach
