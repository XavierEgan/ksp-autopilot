from typing import Literal, Callable
from datetime import datetime, timezone, timedelta

class Logger:
    logger: 'Logger | None' = None
    default_output: Callable[[str], None] = print
    timezone = timezone(timedelta(hours=11))
    
    @staticmethod
    def get() -> 'Logger':
        if not Logger.logger:
            Logger.logger = Logger()
        return Logger.logger
    
    def log(self, message: str, severity: Literal['INFO', 'WARNING', 'ERROR'] = 'INFO'):
        self.default_output(f"[{datetime.now(self.timezone).strftime('%H:%M:%S')}][{severity}] {message}")


if __name__ == "__main__":
    Logger.get().log("test", 'WARNING')