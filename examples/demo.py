"""A typed behavior that needs no robot, ROS node, or hardware."""

from dataclasses import dataclass
from pathlib import Path

from dynamic_stack_decider import DSD, AbstractActionElement, AbstractDecisionElement


@dataclass
class Blackboard:
    ball_seen: bool = False


class BallSeen(AbstractDecisionElement[Blackboard]):
    def perform(self, reevaluate: bool = False) -> str:
        return "YES" if self.blackboard.ball_seen else "NO"

    def get_reevaluate(self) -> bool:
        return True


class TrackBall(AbstractActionElement[Blackboard]):
    def perform(self, reevaluate: bool = False) -> None:
        print("Tracking the ball")


class SearchBall(AbstractActionElement[Blackboard]):
    def perform(self, reevaluate: bool = False) -> None:
        print("Searching for the ball")


def main() -> None:
    blackboard = Blackboard()
    dsd = DSD(blackboard)
    dsd.actions = {"TrackBall": TrackBall, "SearchBall": SearchBall}
    dsd.decisions = {"BallSeen": BallSeen}
    dsd.load_behavior(Path(__file__).with_suffix(".dsd"))
    dsd.update()
    blackboard.ball_seen = True
    dsd.update()


if __name__ == "__main__":
    main()
