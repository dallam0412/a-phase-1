import numpy as np
from queue import PriorityQueue as pq
import pygame
import math

obstacle_space=[]
table=[]
goal_threshold=[]
open_list=pq()
c2c=[]
check_reach=False

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
            
            c_1 = b_1 + cleareance * (math.sqrt(pow(m,2) + 1))
            c_2 = b_1 - cleareance * (math.sqrt(pow(m,2) + 1))
            c_3 = b_2 + cleareance * (math.sqrt(pow(m,2) + 1))
            c_4 = b_2 - cleareance * (math.sqrt(pow(m,2) + 1))
            c_5 = b_3 + cleareance * (math.sqrt(pow(m,2) + 1))
            c_6 = b_3 - cleareance * (math.sqrt(pow(m,2) + 1))
            c_7 = b_4 + cleareance * (math.sqrt(pow(m,2) + 1))
            c_8 = b_4 - cleareance * (math.sqrt(pow(m,2) + 1))

            if  (y - m*x - min(c_1, c_2)) < 0 and (y + m*x - min(c_3, c_4)) < 0 and (y+ m*x - min(c_5, c_6)) > 0 and (y - m*x + min(c_7, c_8)) > 0:
                obstacle_space.append((x,y))

def roundoff(x):
    x_floor=math.floor(x)
    if x-x_floor>0.5:
        if x-x_floor>=0.75:
            x=round(x)
        else:
            x=x_floor+0.5
    else:
        if x-x_floor>=0.25:
            x=x_floor+0.5
        else:
            x=x_floor
    return x


def dist(p1,p2):
    l_2_norm=math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
    l_2_norm=round(l_2_norm,2)
    return l_2_norm


def check_theta(theta):
    if theta>=360:
        theta=theta-360
    if theta<0:
        theta=theta+360
    return theta

def zero_action(node,visited,stepsize,c2c,goal):
    global check_reach
    new_x=node[0]+stepsize*math.cos(np.deg2rad(node[2]))
    new_y=node[1]+stepsize*math.sin(np.deg2rad(node[2]))
    new_theta=check_theta(node[2])
    new_x=roundoff(new_x)
    new_y=roundoff(new_y)
    new_node=(new_x,new_y,new_theta)
    if new_x>0 and new_x<600 and new_y>0 and new_y<250:
        if visited[int(new_x*2)][int(new_y*2)][int(new_theta/30)]!=1 and (new_x,new_y) not in obstacle_space:
            new_c2c=c2c+stepsize
            new_c2g=dist(new_node,goal)
            cost=new_c2c+new_c2g
            for i in range(open_list.qsize()):
                if open_list.queue[i][3]==new_node:
                    if open_list.queue[i][0]>cost:
                        open_list.queue[i]=(cost,new_c2g,new_c2c,new_node)
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
            open_list.put((cost,new_c2g,new_c2c,new_node))
            if dist(new_node,goal)<1.5:
                check_reach=True

def minus60_action(node,visited,stepsize,c2c,goal):
    global check_reach
    new_x=node[0]+stepsize*math.cos(np.deg2rad(node[2]-60))
    new_y=node[1]+stepsize*math.sin(np.deg2rad(node[2]-60))
    new_theta=check_theta(node[2]-60)
    new_x=roundoff(new_x)
    new_y=roundoff(new_y)
    new_node=(new_x,new_y,new_theta)
    if new_x>0 and new_x<600 and new_y>0 and new_y<250:
        if visited[int(new_x*2)][int(new_y*2)][int(new_theta/30)]!=1 and (new_x,new_y) not in obstacle_space:
            new_c2c=c2c+stepsize
            new_c2g=dist(new_node,goal)
            cost=new_c2c+new_c2g
            for i in range(open_list.qsize()):
                if open_list.queue[i][3]==new_node:
                    if open_list.queue[i][0]>cost:
                        open_list.queue[i]=(cost,new_c2g,new_c2c,new_node)
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
            open_list.put((cost,new_c2g,new_c2c,new_node))
            if dist(new_node,goal)<1.5:
                check_reach=True

def minus30_action(node,visited,stepsize,c2c,goal):
    global check_reach
    new_x=node[0]+stepsize*math.cos(np.deg2rad(node[2]-30))
    new_y=node[1]+stepsize*math.sin(np.deg2rad(node[2]))
    new_theta=check_theta(node[2]-30)
    new_x=roundoff(new_x)
    new_y=roundoff(new_y)
    new_node=(new_x,new_y,new_theta)
    if new_x>0 and new_x<600 and new_y>0 and new_y<250:
        if visited[int(new_x*2)][int(new_y*2)][int(new_theta/30)]!=1 and (new_x,new_y) not in obstacle_space:
            new_c2c=c2c+stepsize
            new_c2g=dist(new_node,goal)
            cost=new_c2c+new_c2g
            for i in range(open_list.qsize()):
                if open_list.queue[i][3]==new_node:
                    if open_list.queue[i][0]>cost:
                        open_list.queue[i]=(cost,new_c2g,new_c2c,new_node)
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
            open_list.put((cost,new_c2g,new_c2c,new_node))
            if dist(new_node,goal)<1.5:
                check_reach=True

def plus30_action(node,visited,stepsize,c2c,goal):
    global check_reach
    new_x=node[0]+stepsize*math.cos(np.deg2rad(node[2]+30))
    new_y=node[1]+stepsize*math.sin(np.deg2rad(node[2]+30))
    new_theta=check_theta(node[2]+30)
    new_x=roundoff(new_x)
    new_y=roundoff(new_y)
    new_node=(new_x,new_y,new_theta)
    if new_x>0 and new_x<600 and new_y>0 and new_y<250:
        if visited[int(new_x*2)][int(new_y*2)][int(new_theta/30)]!=1 and (new_x,new_y) not in obstacle_space:
            new_c2c=c2c+stepsize
            new_c2g=dist(new_node,goal)
            cost=new_c2c+new_c2g
            for i in range(open_list.qsize()):
                if open_list.queue[i][3]==new_node:
                    if open_list.queue[i][0]>cost:
                        open_list.queue[i]=(cost,new_c2g,new_c2c,new_node)
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
            open_list.put((cost,new_c2g,new_c2c,new_node))
            if dist(new_node,goal)<1.5:
                check_reach=True

def plus60_action(node,visited,stepsize,c2c,goal):
    global check_reach
    new_x=node[0]+stepsize*math.cos(np.deg2rad(node[2]+60))
    new_y=node[1]+stepsize*math.sin(np.deg2rad(node[2]+60))
    new_theta=check_theta(node[2]+60)
    new_x=roundoff(new_x)
    new_y=roundoff(new_y)
    new_node=(new_x,new_y,new_theta)
    if new_x>0 and new_x<600 and new_y>0 and new_y<250:
        if visited[int(new_x*2)][int(new_y*2)][int(new_theta/30)]!=1 and (new_x,new_y) not in obstacle_space:
            new_c2c=c2c+stepsize
            new_c2g=dist(new_node,goal)
            cost=new_c2c+new_c2g
            for i in range(open_list.qsize()):
                if open_list.queue[i][3]==new_node:
                    if open_list.queue[i][0]>cost:
                        open_list.queue[i]=(cost,new_c2g,new_c2c,new_node)
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
            open_list.put((cost,new_c2g,new_c2c,new_node))
            if dist(new_node,goal)<1.5:
                check_reach=True

cleareance=int(input("input robot clearance "))
radius=int(input("input robot radius "))
while(True):
    stepsize=int(input("input stepsize "))
    if stepsize>=1 and stepsize<=10:
        break

while(True):
    start_x=int(input("input start x coordinate"))
    start_y=int(input("input start y coordinate"))
    start_theta=int(input("input start orientation"))
    if (start_x,start_y) not in obstacle_space and start_theta%30==0:
        start=(start_x,start_y,start_theta)
        break

while(True):
    goal_x=int(input("input goal x coordinate"))
    goal_y=int(input("input goal y coordinate"))
    goal_theta=int(input("input goal orientation"))
    if (goal_x,goal_y) not in obstacle_space and goal_theta%30==0:
        goal=(goal_x,goal_y,goal_theta)
        break

map(radius+cleareance)
open_list.put((dist(start,goal),dist(start,goal),0,start))
closed_list=np.zeros((1200,500,12),dtype=int)
closed_list_vis=[]
new_goal=(0,0,0)
got_goal=0
back_track=[]
table.append([0,start,start])
while(open_list.empty()==False):
    current_node=open_list.get()
    now_node=current_node[3]
    if closed_list[int(now_node[0]*2)][int(now_node[1]*2)][int(now_node[2]/30)]!=1:
        closed_list[int(now_node[0]*2)][int(now_node[1]*2)][int(now_node[2]/30)]=1
        closed_list_vis.append((now_node[0],now_node[1]))
        if dist(now_node,goal)>1.5:
            if check_reach==False and now_node[0]>0 and now_node[0]+(stepsize*math.cos(np.deg2rad(now_node[2]+30)))<600 and now_node[1]>0 and now_node[1]+(stepsize*math.sin(np.deg2rad(now_node[2]+30)))<250:
                plus30_action(now_node,closed_list,stepsize,current_node[2],goal)
            if check_reach==False and now_node[0]>0 and now_node[0]+(stepsize*math.cos(np.deg2rad(now_node[2]+60)))<600 and now_node[1]>0 and now_node[1]+(stepsize*math.sin(np.deg2rad(now_node[2]+60)))<250:
                plus60_action(now_node,closed_list,stepsize,current_node[2],goal)
            if check_reach==False and now_node[0]>0 and now_node[0]+stepsize<600 and now_node[1]>0 and now_node[1]+stepsize<250:
                zero_action(now_node,closed_list,stepsize,current_node[2],goal)
            if check_reach==False and now_node[0]>0 and now_node[0]+(stepsize*math.cos(np.deg2rad(now_node[2]-60)))<600 and now_node[1]>0 and now_node[1]+(stepsize*math.sin(np.deg2rad(now_node[2]-60)))<250:
                minus60_action(now_node,closed_list,stepsize,current_node[2],goal)
            if check_reach==False and now_node[0]>0 and now_node[0]+(stepsize*math.cos(np.deg2rad(now_node[2]-30)))<600 and now_node[1]>0 and now_node[1]+(stepsize*math.sin(np.deg2rad(now_node[2]-30)))<250:
                minus30_action(now_node,closed_list,stepsize,current_node[2],goal)
        else:
            got_goal=1
            new_goal=(now_node[0],now_node[1],now_node[2])
            break
if got_goal==1:
    back_node=new_goal
    back_track=[back_node]
    while(True):
        for i in range(len(table)):
            if table[i][2]==back_node:
                back_node=table[i][1]
                back_track.append(back_node)
                break
        if back_node==start:
            break
    back_track.reverse()
    print(back_track)