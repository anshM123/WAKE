from queue import Queue,Full,Empty
class LatestOnlyQueue:
    """Drops stale visualization work under load."""
    def __init__(self):self.queue=Queue(maxsize=1)
    def put(self,value):
        try:self.queue.put_nowait(value)
        except Full:
            try:self.queue.get_nowait()
            except Empty:pass
            self.queue.put_nowait(value)
    def get(self):return self.queue.get()
