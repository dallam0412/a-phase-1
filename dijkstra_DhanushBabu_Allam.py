
import numpy as np
from queue import PriorityQueue as pq
import pygame
import math
obstacle_space=[]
table=[]
goal_threshold=[]
open_list=pq()
c2c=[]
def map(cleareance):


    for x in range(0,601):
        for y in range(0,cleareance+1):
            obstacle_space.append((x,y))

    for x in range(0,601):
        for y in range(251-cleareance-1,251):
            obstacle_space.append((x,y))

    for x in range(0,cleareance+1):
        for y in range(0,251):
            obstacle_space.append((x,y))

    for x in range(601-cleareance,601):
        for y in range(0,251):
            obstacle_space.append((x,y))
    
    for x in range(100 - cleareance, 150 + cleareance + 1):
        for y in range(0, 250+1):
            if y <= (100 + cleareance) or y >= (150 - cleareance):
                obstacle_space.append((x,y))
    
    for x in range(460-cleareance, 510+2*cleareance):
        for y in range(0, 250):
            m_1, m_2 = 2, 2
            b_1, b_2 = 895, 1145
            if (m_1*x - y <= b_1) and (m_2*x+y <= b_2):
                    obstacle_space.append((x,y))
            
            c_1 = b_1 + cleareance * (math.sqrt(pow(m_2,2) + 1))
            c_2 = b_1 - cleareance * (math.sqrt(pow(m_2,2) + 1))
            c_3 = b_2 + cleareance * (math.sqrt(pow(m_1,2) + 1))
            c_4 = b_2 - cleareance * (math.sqrt(pow(m_1,2) + 1))

            if (m_1*x-y <= min(c_1, c_2)) and (m_2*x+y <= min(c_3, c_4)):
                    obstacle_space.append((x,y))
    for x in range(300 - int(64.95) - cleareance, 300 + int(64.95) + cleareance):
        for y in range(125 - 75 - cleareance, 125 + 75 + cleareance):
            m = 0.577
            b_1, b_2, b_3, b_4 = 32.692, 378.846, 217.307, 128.846
            if  (y - m*x - b_1) < 0 and (y + m*x - b_2) < 0 and (y + m*x - b_3) > 0 and (y - m*x + b_4) > 0:
                obstacle_space.append((x,y))
            
            c_1 = b_1 + clearance * (math.sqrt(pow(m,2) + 1))
            c_2 = b_1 - clearance * (math.sqrt(pow(m,2) + 1))
            c_3 = b_2 + clearance * (math.sqrt(pow(m,2) + 1))
            c_4 = b_2 - clearance * (math.sqrt(pow(m,2) + 1))
            c_5 = b_3 + clearance * (math.sqrt(pow(m,2) + 1))
            c_6 = b_3 - clearance * (math.sqrt(pow(m,2) + 1))
            c_7 = b_4 + clearance * (math.sqrt(pow(m,2) + 1))
            c_8 = b_4 - clearance * (math.sqrt(pow(m,2) + 1))

            if  (y - m*x - min(c_1, c_2)) < 0 and (y + m*x - min(c_3, c_4)) < 0 and (y+ m*x - min(c_5, c_6)) > 0 and (y - m*x + min(c_7, c_8)) > 0:
                obstacle_space.append((x,y))


def zero_action(node,cost,visited,stepsize,c2c,c2g,goal):
    new_x=node[0]+stepsize*math.cos(np.deg2rad(node[2]))
    new_y=node[0]+stepsize*math.sin(np.deg2rad(node[2]))
    new_theta=node[2]+360
    check_new_x=math.floor(new_x)
    check_new_y=math.floor(new_y)
    if new_x-check_new_x>0.5:
        if new_x-check_new_x>=0.75:
            new_x=round(new_x)
        else:
            new_x=math.floor(new_x)+0.5
    else:
        if new_x-check_new_x>=0.25:
            new_x=math.floor(new_x)+0.5
        else:
            new_x=math.floor(new_x)
    if new_y-check_new_y>0.5:
        if new_y-check_new_y>=0.75:
            new_y=round(new_y)
        else:
            new_y=math.floor(new_y)+0.5
    else:
        if new_y-check_new_y>=0.25:
            new_y=math.floor(new_y)+0.5
        else:
            new_y=math.floor(new_y)
    if new_theta>360:
        new_theta=new_theta-360
    new_node=(new_x,new_y,new_theta)
    obstacle_node=(new_x,new_y)
    if visited[new_y*2][new_x*2][360/new_theta]!=1 and obstacle_node not in obstacle_space and new_y<=250:
        c2c=c2c+stepsize
        c2g=math.sqrt((goal[0]-new_x)**2+(goal[1]-new_y)**2)
        cost=c2c+c2g
        for i in range(len(table)):
            if table[i][2]==new_node:
                if cost<table[i][0]:
                    table[i][0]=cost
                    table[i][1]=node
                    table[i][2]=new_node
                    return
                else:
                    return
        table.append([cost,node,new_node])
        open_list.put((cost,(new_node),c2c,c2g))

def minus30_action(node,cost,visited,stepsize,c2c,c2g,goal):
    new_x=node[0]+stepsize*math.cos(np.deg2rad(node[2]-30))
    new_y=node[0]+stepsize*math.sin(np.deg2rad(node[2]-30))
    new_theta=node[2]+360
    check_new_x=math.floor(new_x)
    check_new_y=math.floor(new_y)
    if new_x-check_new_x>0.5:
        if new_x-check_new_x>=0.75:
            new_x=round(new_x)
        else:
            new_x=math.floor(new_x)+0.5
    else:
        if new_x-check_new_x>=0.25:
            new_x=math.floor(new_x)+0.5
        else:
            new_x=math.floor(new_x)
    if new_y-check_new_y>0.5:
        if new_y-check_new_y>=0.75:
            new_y=round(new_y)
        else:
            new_y=math.floor(new_y)+0.5
    else:
        if new_y-check_new_y>=0.25:
            new_y=math.floor(new_y)+0.5
        else:
            new_y=math.floor(new_y)
    if new_theta>360:
        new_theta=new_theta-360
    new_node=(new_x,new_y,new_theta)
    obstacle_node=(new_x,new_y)
    if visited[new_y*2][new_x*2][360/new_theta]!=1 and obstacle_node not in obstacle_space and new_y<=250:
        c2c=c2c+stepsize
        c2g=math.sqrt((goal[0]-new_x)**2+(goal[1]-new_y)**2)
        cost=c2c+c2g
        for i in range(len(table)):
            if table[i][2]==new_node:
                if cost<table[i][0]:
                    table[i][0]=cost
                    table[i][1]=node
                    table[i][2]=new_node
                    return
                else:
                    return
        table.append([cost,node,new_node])
        open_list.put((cost,(new_node),c2c,c2g))

def minus60_action(node,cost,visited,stepsize,c2c,c2g,goal):
    new_x=node[0]+stepsize*math.cos(np.deg2rad(node[2]-60))
    new_y=node[0]+stepsize*math.sin(np.deg2rad(node[2]-60))
    new_theta=node[2]+360
    check_new_x=math.floor(new_x)
    check_new_y=math.floor(new_y)
    if new_x-check_new_x>0.5:
        if new_x-check_new_x>=0.75:
            new_x=round(new_x)
        else:
            new_x=math.floor(new_x)+0.5
    else:
        if new_x-check_new_x>=0.25:
            new_x=math.floor(new_x)+0.5
        else:
            new_x=math.floor(new_x)
    if new_y-check_new_y>0.5:
        if new_y-check_new_y>=0.75:
            new_y=round(new_y)
        else:
            new_y=math.floor(new_y)+0.5
    else:
        if new_y-check_new_y>=0.25:
            new_y=math.floor(new_y)+0.5
        else:
            new_y=math.floor(new_y)
    if new_theta>360:
        new_theta=new_theta-360
    new_node=(new_x,new_y,new_theta)
    obstacle_node=(new_x,new_y)
    if visited[new_y*2][new_x*2][360/new_theta]!=1 and obstacle_node not in obstacle_space and new_y<=250:
        c2c=c2c+stepsize
        c2g=math.sqrt((goal[0]-new_x)**2+(goal[1]-new_y)**2)
        cost=c2c+c2g
        for i in range(len(table)):
            if table[i][2]==new_node:
                if cost<table[i][0]:
                    table[i][0]=cost
                    table[i][1]=node
                    table[i][2]=new_node
                    return
                else:
                    return
        table.append([cost,node,new_node])
        open_list.put((cost,(new_node),c2c,c2g))

def plus60_action(node,cost,visited,stepsize,c2c,c2g,goal):
    new_x=node[0]+stepsize*math.cos(np.deg2rad(node[2]+60))
    new_y=node[0]+stepsize*math.sin(np.deg2rad(node[2]+60))
    new_theta=node[2]+360
    check_new_x=math.floor(new_x)
    check_new_y=math.floor(new_y)
    if new_x-check_new_x>0.5:
        if new_x-check_new_x>=0.75:
            new_x=round(new_x)
        else:
            new_x=math.floor(new_x)+0.5
    else:
        if new_x-check_new_x>=0.25:
            new_x=math.floor(new_x)+0.5
        else:
            new_x=math.floor(new_x)
    if new_y-check_new_y>0.5:
        if new_y-check_new_y>=0.75:
            new_y=round(new_y)
        else:
            new_y=math.floor(new_y)+0.5
    else:
        if new_y-check_new_y>=0.25:
            new_y=math.floor(new_y)+0.5
        else:
            new_y=math.floor(new_y)
    if new_theta>360:
        new_theta=new_theta-360
    new_node=(new_x,new_y,new_theta)
    obstacle_node=(new_x,new_y)
    if visited[new_y*2][new_x*2][360/new_theta]!=1 and obstacle_node not in obstacle_space and new_y<=250:
        c2c=c2c+stepsize
        c2g=math.sqrt((goal[0]-new_x)**2+(goal[1]-new_y)**2)
        cost=c2c+c2g
        for i in range(len(table)):
            if table[i][2]==new_node:
                if cost<table[i][0]:
                    table[i][0]=cost
                    table[i][1]=node
                    table[i][2]=new_node
                    return
                else:
                    return
        table.append([cost,node,new_node])
        open_list.put((cost,(new_node),c2c,c2g))

def plus30_action(node,cost,visited,stepsize,c2c,c2g,goal):
    new_x=node[0]+stepsize*math.cos(np.deg2rad(node[2]+30))
    new_y=node[0]+stepsize*math.sin(np.deg2rad(node[2]+30))
    new_theta=node[2]+360
    check_new_x=math.floor(new_x)
    check_new_y=math.floor(new_y)
    if new_x-check_new_x>0.5:
        if new_x-check_new_x>=0.75:
            new_x=round(new_x)
        else:
            new_x=math.floor(new_x)+0.5
    else:
        if new_x-check_new_x>=0.25:
            new_x=math.floor(new_x)+0.5
        else:
            new_x=math.floor(new_x)
    if new_y-check_new_y>0.5:
        if new_y-check_new_y>=0.75:
            new_y=round(new_y)
        else:
            new_y=math.floor(new_y)+0.5
    else:
        if new_y-check_new_y>=0.25:
            new_y=math.floor(new_y)+0.5
        else:
            new_y=math.floor(new_y)
    if new_theta>360:
        new_theta=new_theta-360
    new_node=(new_x,new_y,new_theta)
    obstacle_node=(new_x,new_y)
    if visited[new_y*2][new_x*2][360/new_theta]!=1 and obstacle_node not in obstacle_space and new_y<=250:
        c2c=c2c+stepsize
        c2g=math.sqrt((goal[0]-new_x)**2+(goal[1]-new_y)**2)
        cost=c2c+c2g
        for i in range(len(table)):
            if table[i][2]==new_node:
                if cost<table[i][0]:
                    table[i][0]=cost
                    table[i][1]=node
                    table[i][2]=new_node
                    return
                else:
                    return
        table.append([cost,node,new_node])
        open_list.put((cost,(new_node),c2c,c2g))

def goal_thresh(x_g,y_g):
    for x in range(600):
        for y in range(250):
            if ((x-x_g)**2+(y-y_g)**2)<=(1.5**2):
                goal_threshold.append((x,y))


def convert_coord(coordinate,frame_height):
    return(coordinate[0],frame_height-coordinate[1])
def convert_rect_coord(coordinate,frame_height,rect_height):
    return(coordinate[0],frame_height-coordinate[1]-rect_height)

def disp(bfs,path):
    pygame.init()
    canvas=pygame.display.set_mode((600,250))
    clock=pygame.time.Clock()
    check=True

    rect_down_space=convert_rect_coord([95,0],250,105)
    rect_up_space=convert_rect_coord([95,145],250,105)
    tri_space_1=convert_coord([455,20],250)
    tri_space_2=convert_coord([463,20],250)
    tri_space_3=convert_coord([515.5,125],250)
    tri_space_4=convert_coord([463,230],250)
    tri_space_5=convert_coord([455,230],250)
    hex_space_1 = convert_coord([300, 205.76], 250)
    hex_space_2 = convert_coord([230, 165.38], 250)
    hex_space_3 = convert_coord([230, 84.61], 250)
    hex_space_4 = convert_coord([300, 44.23], 250)
    hex_space_5 = convert_coord([370, 84.61], 250)
    hex_space_6 = convert_coord([370, 165.38], 250)
    while check:
        for loop in pygame.event.get():
            if loop.type==pygame.QUIT:
                check=False
        pygame.draw.rect(canvas,"yellow",pygame.Rect(rect_down_space[0],rect_down_space[1],60,105))
        pygame.draw.rect(canvas,"yellow",pygame.Rect(rect_up_space[0],rect_up_space[1],60,105))
        pygame.draw.polygon(canvas,"yellow",((tri_space_1),(tri_space_2),(tri_space_3),(tri_space_4),(tri_space_5)))
        pygame.draw.polygon(canvas,"yellow",((hex_space_1),(hex_space_2),(hex_space_3),(hex_space_4),(hex_space_5),(hex_space_6)))
        pygame.draw.rect(canvas,"orange",pygame.Rect(100,0,50,100))
        pygame.draw.rect(canvas,"orange",pygame.Rect(100,150,50,100))
        pygame.draw.polygon(canvas,"orange",((460,25),(460,225),(510,125)))
        pygame.draw.polygon(canvas,"orange",((235,87.5),(300,50),(365,87.5),(365,162.5),(300,200),(235,162.5)))
        pygame.draw.rect(canvas,"yellow",pygame.Rect(0,0,5,250))
        pygame.draw.rect(canvas,"yellow",pygame.Rect(595,0,5,250))
        pygame.draw.rect(canvas,"yellow",pygame.Rect(0,0,600,5))
        pygame.draw.rect(canvas,"yellow",pygame.Rect(0,245,600,5))

        for i in bfs:
            pygame.draw.circle(canvas,"white",convert_coord(i,250),1)
            pygame.display.flip()
            clock.tick(1000)
        for i in path:
            pygame.draw.circle(canvas,"black",convert_coord(i,250),1)
            pygame.display.flip()
            clock.tick(20)
        pygame.display.flip()
        pygame.time.wait(5000)
        check=False
        pygame.quit()


clearance=int(input("enter the clearance of the robot"))
while(True):
    stepsize=int(input("stepsize"))
    if stepsize>=1 and stepsize<=10:
        break
robot_radius=int(input("enter robot radius"))
map(clearance)
check_correct_input=False
while (check_correct_input!=True):
    start_x=int(input("enter the x coordinate of start node "))
    start_y=int(input("enter the y coordinate of start node "))
    start_theta=int(input("enter the starting orientation"))
    if start_theta==0:
        start_theta=360
    start=(start_x,start_y,start_theta)
    if start in obstacle_space:
        print("start node is in obstacle space")
        continue
    else:
        check_correct_input=True
    goal_x=int(input("enter the x coordinate of goal node "))
    goal_y=int(input("enter the y coordinate of goal node "))
    goal_orientation=int(input("enter the goal orientation"))
    goal=(goal_x,goal_y,goal_orientation)
    if goal in obstacle_space:
        check_correct_input=False
        print("end node is in obstacle space")
goals=goal_thresh(goal_x,goal_y)
open_list.put((0,start,0,0))
table.append([0,(start_x,start_y),(start_x,start_y)])
closed_list=np.zeros((500,1200,13),dtype="int")
got_goal=0
print("processing")
while(open_list.empty()==False):
    current_node=open_list.get()
    now_node=current_node[1]
    x=now_node[0]
    y=now_node[1]
    the=now_node[2]
    if ((x-goal_x)**2+(y-goal_y)**2)<=(1.5**2):
        got_goal=1
        break
    closed_list[y*2][x*2][360/the]=1
    zero_action(current_node[1],current_node[0],closed_list,stepsize,current_node[2],current_node[3],goal)
    minus60_action(current_node[1],current_node[0],closed_list,stepsize,current_node[2],current_node[3],goal)
    minus30_action(current_node[1],current_node[0],closed_list,stepsize,current_node[2],current_node[3],goal)
    plus30_action(current_node[1],current_node[0],closed_list,stepsize,current_node[2],current_node[3],goal)
    plus60_action(current_node[1],current_node[0],closed_list,stepsize,current_node[2],current_node[3],goal)
closed_list.append(goal)


