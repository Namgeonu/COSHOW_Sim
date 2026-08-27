
class ActionClient:
    def __init__(self, node, t, name): self.name=name; self.node=node
    def wait_for_server(self, timeout_sec=0.0): return True
    def send_goal_async(self, goal):
        self.node.log.append(('nav', self.name, goal.pose.pose.position.x, goal.pose.pose.position.y))
        return self.node.make_goal_future(self.name, goal)
