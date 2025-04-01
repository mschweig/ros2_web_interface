from abc import ABC, abstractmethod
from typing import Any

class ROSInterface(ABC):
    @abstractmethod
    def call(self, name: str, data: Any = None) -> Any:
        pass