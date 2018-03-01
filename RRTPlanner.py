import numpy
from RRTTree import RRTTree


class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.tree = []
        self.path = []
        self.result = []
        self.arrived = False


    def Plan(self, start_config, goal_config, epsilon = 1.0):
        self.tree = RRTTree(self.planning_env, start_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        goal_tf = numpy.eye(4,dtype = float)
        goal_tf[0,3] = goal_config[0]
        goal_tf[1,3] = goal_config[1]

        start_tf = numpy.eye(4,dtype = float)
        start_tf[0,3] = start_config[0]
        start_tf[1,3] = start_config[1]

        # plan.append(start_config)
        # plan.append(goal_config)

        self.tree.vertices = []
        # print("initial tree vertices is {} ".format(tree.vertices))
        self.tree.AddVertex(start_tf)
        # print("current tree vertices is {} ".format(tree.vertices))
        count = 0

        while (True):
            count += 1
            # Generate random point
            if(count % 10 is not 0):
                qr = self.planning_env.GenerateRandomConfiguration()
            else:
                qr = goal_tf
            # Find the nearest point to the random point
            # qi: nearest point
            if(numpy.shape(qr) != (4, 4)):
                qr = self.planning_env.GenerateRandomConfiguration()
                qi_id, qi = self.tree.GetNearestVertex(qr)
            else:
                qi_id, qi = self.tree.GetNearestVertex(qr)

            # Extend the new point from the nearest point toward the random point for one step distance
            qc = self.planning_env.Extend(qi,qr)

            qc_id = self.tree.AddVertex(qc)
            self.tree.AddEdge(qi_id, qc_id)

            # when reach the goal, break the while loop
            if(self.planning_env.ComputeDistance(qc, goal_tf) < epsilon):
                print("Reach the goal is {} ".format(qc_id))
                break

        # use DFS to find the path from the start_config to the goal_config
        # print("Print all paths is {} ".format(self.tree.vertices))

        visited = [False] * len(self.tree.vertices)
        # self.path = self.printAllPaths(1,qc_id)

        path = []
        level = 0
        self.printAllPathsUtil(0, qc_id,visited,path,level)

        print("Not self path is {}".format(self.path))
        idx = 0
        while(self.path[idx] is not qc_id):
            self.result.append(self.path[idx])
            idx += 1
        self.result.append(qc_id)
        print("The self result is {}".format(self.result))

        for i in self.result:
            v = self.tree.vertices[i]
            plan.append((v[0,3],v[1,3]))
        print("Final plan is {}".format(plan))
        print("Goal Config is {} ".format(goal_config))
        return plan

    # def printAllPaths(self,s,d):
    #     # Mark all the vertices as not visited
    #     visited = [False] * len(self.tree.vertices)
    #     self.path = []
    #     self.path = self.printAllPathsUtil(s,d,visited,self.path)
    #     print("Final path is {}".format(self.path))
    #     return

    def printAllPathsUtil(self, u, d, visited, path,level):
        level += 1
        visited[u] = True
        path.append(u)
        if u == d:
            print("---------------------------------------------")
            print("This is path {}".format(path))
            # self.result.append(path)
            self.path = path
        else:
            for i in self.tree.edges[u]:
                if visited[i] == False:
                    print("level %d, val %d "%(level,i))
                    self.printAllPathsUtil(i,d,visited,path,level)
        # path.pop()
        # visited[u] = False
        # print("self path {} ".format(self.path))
        # print("---- path {} ".format(path))
        # return path

    # def DFS(self,v):
    #     # Mark all the vertices as not visited
    #     visited = [False]*(len(self.graph))
    #     self.DFSUtil(v,visited)
    #
    # def DFSUtil(self,v,visited):
    #
    #     # Mark the current node as visited and print
    #     visited[v] = True
    #     for i in tree.edges[v]:
    #         if visited[i] == False:
    #             self.DFSUtil(i,visited)
