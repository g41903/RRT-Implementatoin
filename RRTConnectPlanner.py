import numpy, operator
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []
        #true represents ftree, false represents rtree
        isftree = True
        qi_f = start_config
        qi_r = goal_config

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        while(True):
            if(isftree):
                #update qi_f
                qi_f_id, qi_f = self.ftree.GetNearestVertex(qi_r)
                qr_id, qr = self.rtree.GetNearestVertex(qi_f)
                if(self.planning_env.Extend(qi_f,qr)== qi_f):
                    qr = self.planning_env.GenerateRandomConfiguration()
                qc = self.planning_env.Extend(qi_f, qr)
                qc_id = self.ftree.AddVertex(qc)
                self.ftree.AddEdge(qi_f_id, qc_id)




            if(len(self.ftree.vertices) > len(self.rtree.vertices)):
                isftree = False

            if (!isftree):
                # update qi_r
                qi_r_id, qi_r = self.rtree.GetNearestVertex(qi_f)
                qr_id, qr = self.ftree.GetNearestVertex(qi_r)
                if (self.planning_env.Extend(qi_r, qr) == qi_r):
                    qr = self.planning_env.GenerateRandomConfiguration()
                qc = self.planning_env.Extend(qi_r, qr)
                qc_id = self.rtree.AddVertex(qc)
                self.rtree.AddEdge(qi_r_id, qc_id)

            if (len(self.ftree.vertices) < len(self.rtree.vertices)):
                isftree = True

            if(self.planning_env.ComputeDistance(qi_r, qi_f) < epsilon):
                print("Reach the goal ")
                break

        plan.append(start_config)
        plan.append(goal_config)
        
        return plan
