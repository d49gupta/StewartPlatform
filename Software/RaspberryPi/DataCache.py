from loggingModule import logger as lg
class CircularBuffer:
    def __init__(self, size):
        self.buffer = [None] * size
        self.size = size
        self.head = 0  # Write pointer (newest element in data queue)
        self.tail = 0  # Read pointer (oldest element in data queue)
        self.count = 0  # Track number of elements in the buffer

    def is_full(self):
        return self.count == self.size

    def is_empty(self):
        return self.count == 0

    def enqueue(self, data):
        self.buffer[self.head] = data
        self.head = (self.head + 1) % self.size
        if self.is_full():
            self.tail = (self.tail + 1) % self.size
        else:
            self.count += 1

    def dequeue(self):
        if not self.is_empty():
            data = self.buffer[self.tail]
            self.buffer[self.tail] = None
            self.tail = (self.tail + 1) % self.size
            self.count -= 1
            return data
        else:
            lg.fatal("Buffer is empty. Cannot dequeue.")
            return None

    def oldestValue(self):
        if not self.is_empty():
            return self.buffer[self.tail]
        else:
            lg.warning("Buffer is empty. Nothing to peek at.")
            return None
    
    def newestValue(self):
        if not self.is_empty():
            newest_index = (self.head - 1) % self.size
            return self.buffer[newest_index]
        else:
            lg.warning("Buffer is empty. Nothing to peek at.")
            return None

    def __str__(self):
        return f"CircularBuffer({self.buffer})" # used to print object CircularBuffer

if __name__ == '__main__':
    buffer_size = 5
    cb = CircularBuffer(buffer_size)

    cb.enqueue(10)
    cb.enqueue(20)
    cb.enqueue(30)
    print(cb)

    print("Dequeued:", cb.dequeue())
    print(cb)

    print("Peek:", cb.oldestValue())
    cb.enqueue(49)

    print("Snoop:", cb.newestValue())
    print(cb)