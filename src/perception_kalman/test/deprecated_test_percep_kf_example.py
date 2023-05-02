# import pytest
from perception_kalman.perception_kalman import determine_lane


def test_determine_lane_func():
    assert (determine_lane(3)) == (3, False)
