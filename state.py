class STATE:
    def __init__(self) -> None:
        self.workmode: str = "init"
        self.lock: bool = True
        self.speaker: bool = False
        self.connected: bool = False
        self.sleep: bool = False
        self.running: bool = True
