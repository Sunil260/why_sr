from dataclasses import dataclass, field
from typing import List, Dict

@dataclass
class RobotState:
    target_ms: float = 0.3
    history: List[Dict] = field(default_factory=list)
    max_history: int = 100
    
    def add_point(self, t, target, actual):
        self.history.append({"t": t, "target": target, "actual": actual})
        if len(self.history) > self.max_history:
            self.history.pop(0)

state = RobotState()