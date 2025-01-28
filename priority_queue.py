class Priority:
    """
    Handle lexicographic ordering of keys for D* Lite algorithm
    """
    def __init__(self, k1: float, k2: float):
        self.k1 = k1
        self.k2 = k2
    
    def __lt__(self, other):
        return self.k1 < other.k1 or (self.k1 == other.k1 and self.k2 < other.k2)
    
    def __le__(self, other):
        return self.k1 < other.k1 or (self.k1 == other.k1 and self.k2 <= other.k2)

class PriorityQueue:
    """
    Priority Queue implementation for D* Lite algorithm
    """
    def __init__(self):
        self.heap = []
        self.vertices_in_heap = set()
    
    def top(self):
        """Get the top vertex without removing it"""
        if not self.heap:
            return None
        return self.heap[0][1]
    
    def top_key(self):
        """Get the top priority without removing it"""
        if not self.heap:
            return Priority(float('inf'), float('inf'))
        return self.heap[0][0]
    
    def pop(self):
        """Remove and return the top vertex"""
        if not self.heap:
            return None
        vertex = self.heap[0][1]
        self.vertices_in_heap.remove(vertex)
        if len(self.heap) > 1:
            self.heap[0] = self.heap[-1]
            self.heap.pop()
            self._sift_down(0)
        else:
            self.heap.pop()
        return vertex
    
    def insert(self, vertex, priority):
        """Insert a vertex with given priority"""
        if vertex in self.vertices_in_heap:
            self.update(vertex, priority)
            return
        self.heap.append((priority, vertex))
        self.vertices_in_heap.add(vertex)
        self._sift_up(len(self.heap) - 1)
    
    def update(self, vertex, priority):
        """Update priority of existing vertex"""
        for i, (_, v) in enumerate(self.heap):
            if v == vertex:
                self.heap[i] = (priority, vertex)
                self._sift_up(i)
                self._sift_down(i)
                return
    
    def remove(self, vertex):
        """Remove a vertex from queue"""
        for i, (_, v) in enumerate(self.heap):
            if v == vertex:
                self.vertices_in_heap.remove(vertex)
                if i < len(self.heap) - 1:
                    self.heap[i] = self.heap[-1]
                    self.heap.pop()
                    self._sift_down(i)
                else:
                    self.heap.pop()
                return
    
    def _sift_up(self, idx):
        """Restore heap property by moving element up"""
        parent = (idx - 1) // 2
        while idx > 0 and self.heap[idx][0] < self.heap[parent][0]:
            self.heap[idx], self.heap[parent] = self.heap[parent], self.heap[idx]
            idx = parent
            parent = (idx - 1) // 2
    
    def _sift_down(self, idx):
        """Restore heap property by moving element down"""
        while True:
            smallest = idx
            left = 2 * idx + 1
            right = 2 * idx + 2
            
            if left < len(self.heap) and self.heap[left][0] < self.heap[smallest][0]:
                smallest = left
            if right < len(self.heap) and self.heap[right][0] < self.heap[smallest][0]:
                smallest = right
                
            if smallest == idx:
                break
                
            self.heap[idx], self.heap[smallest] = self.heap[smallest], self.heap[idx]
            idx = smallest
    
    def __contains__(self, vertex):
        return vertex in self.vertices_in_heap

