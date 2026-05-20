from __future__ import annotations


class FlightError(RuntimeError):
    """Base exception for peregrine flight stack operations."""

    def __init__(self, operation: str, message: str, *, result=None):
        self.operation = operation
        self.result = result
        super().__init__(f"{operation}: {message}")


class ReadinessTimeout(FlightError):
    """UAVState.dependencies_ready did not become True within the timeout."""

    def __init__(self, timeout_s: float, detail: str = ""):
        msg = f"not ready after {timeout_s:.1f}s"
        if detail:
            msg += f" ({detail})"
        super().__init__("wait_ready", msg)


class ActionTimeout(FlightError):
    """An action goal did not complete within the timeout."""

    def __init__(self, operation: str, phase: str):
        super().__init__(operation, f"{phase} timed out")


class ActionRejected(FlightError):
    """An action server rejected the goal."""

    def __init__(self, operation: str):
        super().__init__(operation, "goal rejected by server")


class ActionFailed(FlightError):
    """An action completed but reported failure."""

    def __init__(self, operation: str, message: str, *, result=None):
        super().__init__(operation, message, result=result)


class ServiceError(FlightError):
    """A service call failed."""

    def __init__(self, operation: str, message: str):
        super().__init__(operation, message)
