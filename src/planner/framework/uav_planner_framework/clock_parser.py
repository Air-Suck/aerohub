"""Parse the simulation-time block emitted by ``gz topic -e``."""


class ClockTextParser:
    def __init__(self) -> None:
        self.in_sim = False
        self.sec = 0
        self.nanosec = 0

    def feed(self, line: str) -> tuple[int, int] | None:
        value = line.strip()
        if value == "sim {":
            self.in_sim = True
            self.sec = 0
            self.nanosec = 0
        elif self.in_sim and value.startswith("sec:"):
            self.sec = int(value.split(":", 1)[1])
        elif self.in_sim and value.startswith("nsec:"):
            self.nanosec = int(value.split(":", 1)[1])
        elif self.in_sim and value == "}":
            self.in_sim = False
            return self.sec, self.nanosec
        return None
