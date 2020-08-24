import math
import random
import numpy as np 
import matplotlib.pyplot as plt
import time
import csv

class Node:
    def __init__(self,x,y,nome):
        self.nome = nome
        self.x = x
        self.y = y
        self.pathx = []
        self.pathy = []
        self.parent = None

def draw_graph(rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in nodelist:
            if node.parent:
                plt.plot(node.pathx, node.pathy, "-g")

        for (ox, oy, size) in obstaclelist:
            plot_circle(ox, oy, size)

        plt.plot(start.x, start.y, "xr")
        plt.plot(goal.x, goal.y, "xr")
        plt.axis("equal")
        plt.axis([-0.5, 0.5, -0.5, 0.5])
        plt.grid(True)
        plt.pause(0.0001)

def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

def get_random_node():
    rnd = Node(random.uniform(randomlimit[0], randomlimit[1]), random.uniform(randomlimit[0], randomlimit[1]),'random')

    return rnd

def get_nearest_node_index(nodelist,random_node):
        dlist = [(node.x - random_node.x) ** 2 + (node.y - random_node.y)
                 ** 2 for node in nodelist]
        nearnodeindex = dlist.index(min(dlist))

        return nearnodeindex

def calculadoradistang(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

def grow(from_node,to_node,extensao):
    newnode = Node(from_node.x,from_node.y, from_node.nome)
    d,Theta = calculadoradistang(newnode,to_node)

    newnode.pathx = [newnode.x]
    newnode.pathy = [newnode.y]


    if extensao > d:
            extensao = d

    n_expand = math.floor(extensao / resolucao)

    for _ in range(n_expand):
            newnode.x += resolucao * math.cos(Theta)
            newnode.y += resolucao * math.sin(Theta)
            newnode.pathx.append(newnode.x)
            newnode.pathy.append(newnode.y)  
    
    if d <= resolucao:
        newnode.pathx.append(to_node.x)
        newnode.pathy.append(to_node.y)

    
       
    newnode.parent = from_node
    

    return newnode

def checar_colisão(node,obstaclelist):
    for (ox, oy, size) in obstaclelist:
            dx_list = [ox - x for x in node.pathx]
            dy_list = [oy - y for y in node.pathy]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= (size) ** 2:
                return False  # collision

    return True  # safe

def calc_dis_to_goal(x,y):
    dx = x - goal.x
    dy = y - goal.y
    dist = math.hypot(dx,dy)
    return dist

def GerarCaminho(nodelist):
    path = [[goal.x, goal.y]]
    caminho = [goal.nome]
    node = nodelist[len(nodelist)-1]
    while node.parent is not None:
        path.append([node.x, node.y])
        caminho.append(node.nome)
        node = node.parent
    path.append([node.x, node.y])
    caminho.append(node.nome)
    return path, caminho

def RRT():
    k = 2
    for i in range(maxtreesize):
        if i % 10 == 0:
            random_node = Node(goal.x,goal.y,'random')
        else:
            random_node = get_random_node()
            
        
        nearest_ind = get_nearest_node_index(nodelist, random_node)
        nearest_node = nodelist[nearest_ind]
        new_node = grow(nearest_node, random_node, distancia)

        if checar_colisão(new_node, obstaclelist):
            fromnodenome = new_node.nome
            new_node.nome = str(k)
            k = k+1
            nodelist.append(new_node)
            listaparentesco.append([new_node.nome,fromnodenome,0.1])

        if i % 20 == 0:
            draw_graph(random_node)

        if calc_dis_to_goal(nodelist[-1].x,nodelist[-1].y) <= distancia:
            final_node = grow(nodelist[-1], goal, distancia)

            if checar_colisão(final_node, obstaclelist):
                for j in range(len(nodelist)):
                   listanode.append([nodelist[j].nome, nodelist[j].x, nodelist[j].y,0.1])
                   
                goal.nome = str(len(nodelist) + 1)
                listanode.append([goal.nome,goal.x,goal.y,0.1])
                path, caminho = GerarCaminho(nodelist)
                return path, caminho, listanode, listaparentesco


        #if i % 5:
        #    draw_graph(random_node)
    
    return None

with open('obstacles.csv', 'r') as file:
    reader = csv.reader(file)
    lista = []
    for row in reader:
        row[0] = float(row[0])
        row[1] = float(row[1])
        row[2] = float(row[2])/2
        lista.append(tuple(row))
    

maxtreesize = 1000
start = [-0.5,-0.5]
goal = [0.5,0.5]
obstaclelist = lista
randomlimit = [-0.5,0.5]
nodelist = []
listanode = []
parentesco = []
listaparentesco = []
resolucao = 0.05
distancia = 0.05
start = Node(start[0],start[1], '1')
goal = Node(goal[0], goal[1],'')
nodelist = [start]


path, caminho= None, []

if RRT() is None:
    print('Não foi possivel achar o caminho!')
else:
    path, caminho, listanode, listaparentesco = RRT()


if path is None:
    print("Cannot find path")
else:
    print("found path!!")
    caminho.reverse()
    print(caminho)
    draw_graph()
    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    plt.grid(True)
    plt.pause(0.00001)  # Need for Mac
    plt.show()

with open('path.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            
            writer.writerow(caminho)

with open('nodes.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(listanode)

with open('edges.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(listaparentesco)




    


    








