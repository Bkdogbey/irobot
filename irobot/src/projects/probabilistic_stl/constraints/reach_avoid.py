from pdstl.operators import Always, Eventually, And
from planning.environment import RectangularGoalPredicate


def build_reach_avoid_spec(env, T: int):
    """
    phi = Eventually[0,T](reach goal) AND Always[0,T](avoid all obstacles)

    Test 5 spec: goal + obstacle avoidance.
    """
    preds = env.get_predicates()
    assert preds["goal"] is not None, "Environment must have a goal set"
    assert len(preds["obstacles"]) > 0, "Environment must have at least one obstacle"

    phi_reach = Eventually(preds["goal"], interval=[0, T])

    obs_list = preds["obstacles"]
    phi_safe = obs_list[0]
    for obs in obs_list[1:]:
        phi_safe = And(phi_safe, obs)
    phi_safety = Always(phi_safe, interval=[0, T])

    return And(phi_reach, phi_safety)
