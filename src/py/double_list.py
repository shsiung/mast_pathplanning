import sys

class Node(object):

    def __init__(self, data, prev, next):
        self.data = data
        self.prev = prev
        self.next = next


class DoubleList(object):

    head = None
    tail = None


    def append(self, data):
        new_node = Node(data, None, None)
        if self.head is None:
            self.head = self.tail = new_node
        else:
            new_node.prev = self.tail
            new_node.next = None
            self.tail.next = new_node
            self.tail = new_node


    def replace(self, currentNode, newData):
        # TODO: Fix date to data
        currentNode.data = newData


    def remove(self, current_node):

        # singleton case
        if current_node.prev is None and current_node.next is None:
            self.head = None
            self.tail = None

        # current is head
        elif current_node.prev is None:
            self.head = current_node.next
            current_node.next.prev = None

        # current is tail
        elif current_node.next is None:
            self.tail = current_node.prev
            current_node.prev.next = None

        # otherwise we have no prev (it's None), head is the next one, and prev becomes None
        else:
            current_node.prev.next = current_node.next
            current_node.next.prev = current_node.prev

    def show(self):
        current_node = self.head
        sys.stdout.write('[')
        while current_node is not None:
            sys.stdout.write(str(current_node.data))
            current_node = current_node.next

            if current_node is not None:
                sys.stdout.write(', ')

        sys.stdout.write(']\n')

#dbl = DoubleList()
#for x in range(10):
#    dbl.append(x)
#
#currentNode = dbl.head
#
#dbl.remove(dbl.tail)
#dbl.show()
#
#dbl.remove(dbl.tail)
#dbl.show()
#
#dbl.remove(dbl.tail.prev)
#dbl.show()
