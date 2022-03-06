
import numpy as np
import matplotlib.pyplot as plt
import time
import heapq



class Node:

    def __init__(self, x, y, cost, parent_id):

        self.x = x
        self.y = y
        self.cost = cost
        self.parent_id = parent_id
    
    def __lt__(self,other):
        return self.cost < other.cost

def Actionmove_right(x,y,cost):
    x = x + 1
    cost = 1 + cost
    return x,y,cost

def Actionmove_left(x,y,cost):
    x = x - 1
    cost = 1 + cost
    return x,y,cost

def Actionmove_up(x,y,cost):
    y = y + 1
    cost = 1 + cost
    return x,y,cost

def Actionmove_down(x,y,cost):
    y = y - 1
    cost = 1 + cost
    return x,y,cost

def Actionmove_upright(x,y,cost):
    x = x + 1
    y = y + 1
    cost = np.sqrt(2) + cost
    return x,y,cost

def Actionmove_upleft(x,y,cost):
    x = x - 1
    y = y + 1
    cost = np.sqrt(2) + cost
    return x,y,cost

def Actionmove_downright(x,y,cost):
    x = x + 1
    y = y - 1
    cost = np.sqrt(2) + cost
    return x,y,cost

def Actionmove_downleft(x,y,cost):
    x = x -1
    y = y - 1
    cost = np.sqrt(2) + cost
    return x,y,cost

def Next_Node(move,x,y,cost):

    if move == 'Left':
        return Actionmove_left(x,y,cost)
    elif move == 'Right':
        return Actionmove_right(x,y,cost)
    elif move == 'Up':
        return Actionmove_up(x,y,cost)
    elif move == 'Down':
        return Actionmove_down(x,y,cost)
    elif move == 'UpRight':
        return Actionmove_upright(x,y,cost)
    elif move == 'UpLeft':
        return Actionmove_upleft(x,y,cost)
    elif move == 'DownRight':
        return Actionmove_downright(x,y,cost)
    elif move == 'DownLeft':
        return Actionmove_downleft(x,y,cost)
    else:
        return None


def ObstacleMap(width,height):

    map = np.full((height,width),0)
    r = 40
    for y in range(height):
        for x in range(width):
        
        #Polygon Obstacle (clearance)
            s1 = (y-5) - ((0.316) *(x+5)) - 173.608  
            s2 = (y+5) + (1.23 * (x+5)) - 229.34 
            s3 = (y-5) + (3.2 * (x-5)) - 436 
            s4 = (y+5) - 0.857*(x-5) - 111.42 
            s5 = y + (0.1136*x) - 189.09
        
        #Circle Obstacle (clearance)
            C = ((y -185)**2) + ((x-300)**2) - (r+5)**2  
            
        #Hexagon Obstacle (clearance)
            h1 = (y-5) - 0.577*(x+5) - 24.97
            h2 = (y-5) + 0.577*(x-5) - 255.82
            h3 = (x-6.5) - 235 
            h6 = (x+6.5) - 165 
            h5 = (y+5) + 0.577*(x+5) - 175 
            h4 = (y+5) - 0.577*(x-5) + 55.82 
        
            if(h1<0 and h2<0 and h3<0 and h4>0 and h5>0 and h6>0) or C<=0  or (s1<0 and s5>0 and s4>0)or (s2>0 and s5<0 and s3<0):
                map[y,x]=1
        

            a1 = y - 0.577*x - 24.97 
            a2 = y + 0.577*x - 255.82
            a3 = x - 235 
            a6 = x - 165 
            a5 = y + 0.577*x - 175 
            a4 = y - 0.577*x + 55.82 
        
            D = ((y -185)**2) + ((x-300)**2) - (r)**2 
        
            l1 = y - ((0.316) *x) - 173.608  
            l2 = y + (1.23 * x) - 229.34 
            l3 = y + (3.2 * x) - 436 
            l4 = y - 0.857*x - 111.42 
            l5 = y + (0.1136*x) - 189.09
        
            if(a1<0 and a2<0 and a3<0 and a4>0 and a5>0 and a6>0) or D<0 or (l1<0 and l5>0 and l4>0)or (l2>0 and l5<0 and l3<0): 
                map[y,x]=2

    for i in range(400):
        map[0][i] = 1
        map[1][i] = 1
        map[2][i] = 1
        map[3][i] = 1
        map[4][i] = 1

    for i in range(400):
        map[249][i] = 1
        map[248][i] = 1
        map[247][i] = 1
        map[246][i] = 1
        map[245][i] = 1

    for i in range(250):
        map[i][0] = 1
        map[i][1] = 1
        map[i][2] = 1
        map[i][3] = 1
        map[i][4] = 1

    for i in range(250):
        map[i][399] = 1
        map[i][398] = 1
        map[i][397] = 1
        map[i][396] = 1
        map[i][395] = 1


    return map


def is_goal(current, goal):

    if (current.x == goal.x) and (current.y == goal.y):
        return True
    else:
        return False


def is_valid(x, y, obstacle_map):

    s = obstacle_map.shape

    if( x > s[1] or x < 0 or y > s[0] or y < 0 ):
        return False
    
    else:
        try:
            if(obstacle_map[y][x] == 1)or (obstacle_map[y][x] == 2):
                return False
        except:
            pass
    return True

def Dijkstra_algorithm(start_node, goal_node,obstacle_map):

    if is_goal(start_node, goal_node):
        return None,1
    goal_node = goal_node
    start_node = start_node
    actions = ['Left', 'Right','Up','Down','UpRight','UpLeft','DownRight','DownLeft']
    open_nodes = {}
    open_nodes[(start_node.x * 500+ start_node.y)] = start_node
    closed_nodes = {}
    open_nodes_list = []  
    all_nodes = []
    heapq.heappush(open_nodes_list, [start_node.cost, start_node])

    while (len(open_nodes_list) != 0):

        current_node = (heapq.heappop(open_nodes_list))[1]
        all_nodes.append([current_node.x, current_node.y])
        current_id = (current_node.x * 500 + current_node.y) 

        if is_goal(current_node, goal_node):
            goal_node.parent_id = current_node.parent_id
            goal_node.cost = current_node.cost
            print("Goal Node found")
            return all_nodes,1

        if current_id in closed_nodes:  
            continue
        else:
            closed_nodes[current_id] = current_node
        
        del open_nodes[current_id]

        for move in actions:
            x,y,cost = Next_Node(move,current_node.x,current_node.y,current_node.cost)

            new_node = Node(x,y,cost,current_node)  

            new_node_id = (new_node.x * 500+ new_node.y) 

            if not is_valid(new_node.x, new_node.y, obstacle_map):
                continue
            elif new_node_id in closed_nodes:
                continue

            if new_node_id in open_nodes:
                if new_node.cost < open_nodes[new_node_id].cost:
                    open_nodes[new_node_id].cost = new_node.cost
                    open_nodes[new_node_id].parent_id = new_node.parent_id
            else:
                open_nodes[new_node_id] = new_node
                heapq.heappush(open_nodes_list, [ open_nodes[new_node_id].cost, open_nodes[new_node_id]])

    return  all_nodes,0

def Generate_Path(goal_node):  
    x_path = []
    y_path = []
    x_path.append(goal_node.x)
    y_path.append(goal_node.y)

    parent = goal_node.parent_id
    while parent != -1:
        x_path.append(parent.x)
        y_path.append(parent.y)
        parent = parent.parent_id
    
    return x_path,y_path


def plot(start_node,goal_node,x_path,y_path,all_nodes,obstacle_map):

    plt.plot(start_node.x, start_node.y, "Dw")
    plt.plot(goal_node.x, goal_node.y, "Dg")
    plt.imshow(obstacle_map,"Greens")
    ax = plt.gca()
    ax.invert_yaxis()

    x_path.reverse()
    y_path.reverse()

    for i in range(len(all_nodes)):
        plt.plot(all_nodes[i][0], all_nodes[i][1], "3c")
        plt.pause(0.000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001)

    plt.plot(x_path,y_path,"-r")
    
    plt.show()
    plt.pause(3)
    plt.close('all')

if __name__ == '__main__':

    width = 400
    height = 250
    obstacle_map = ObstacleMap(width, height)

    start_coord = input("Enter start coordinates: ")
    start_x, start_y = start_coord.split()
    start_x = int(start_x)
    start_y = int(start_y)
    if not is_valid(start_x, start_y, obstacle_map):
        print("In valid start node or in Obstacle space")
        exit(-1)
    
    goal_coordinates = input("Enter goal coordinates: ")
    goal_x, goal_y = goal_coordinates.split()
    goal_x = int(goal_x)
    goal_y = int(goal_y)
    if not is_valid(goal_x, goal_y, obstacle_map):
        print("In valid goal node or in Obstacle space")
        exit(-1)
    
    start_node = Node(start_x, start_y, 0.0, -1)
    goal_node = Node(goal_x, goal_y, 0.0, -1)
    all_nodes,flag = Dijkstra_algorithm(start_node, goal_node,obstacle_map)

    if (flag)==1:
        x_path,y_path = Generate_Path(goal_node)
    else:
        print("not found")
        exit(-1)

    plot(start_node,goal_node,x_path,y_path,all_nodes,obstacle_map)
    
    



    








