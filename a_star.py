import numpy as np
from queue import PriorityQueue as pq
import pygame as pg
import math
import matplotlib.pyplot as plt
from sortedcollections import OrderedSet

obstacle_space=OrderedSet()
table=[]
goal_threshold=[]
open_list=pq()
c2c=[]
occupied_space_x=[]
occupied_space_y=[]
check_reach=False

def change_points(points, length):
    return (points[0], length - points[1])

def change_points_rect(points, length, obj_height):
    return (points[0], length - points[1] - obj_height)

def intersect_detect(slope_1, slope_2, const_1, const_2, coeff_1, coeff_2):

    first = np.array([[-slope_1, coeff_1], [-slope_2, coeff_2]])
    second = np.array([const_1, const_2])
    combined = np.linalg.solve(first, second)
    return combined

def parallel_line_finder(slope, const, bloat, str):
    first = const + bloat*np.sqrt(slope**2 + 1)
    second = const - bloat*np.sqrt(slope**2 + 1)
    if str == 'upper':

        if first > const:
            return first
        else:

            return second
    else:

        if first < const:

            return first
        else:
            return second

def clearance(bloat, checker):

    hexagon_coeff_1 = hexagon_coeff_3 = -15/26
    hexagon_coeff_2 = hexagon_coeff_4 =  15/26

    hexagon_const_1, hexagon_const_2, hexagon_const_3  = 4850/13, 350/13, 2900/13
    hexagon_const_4, hexagon_const_5, hexagon_const_6 = -1600/13, 235, 365

    triangle_coeff_1 = -2
    triangle_coeff_2 =  2

    triangle_const_1 = 1145
    triangle_const_2 = -895
    triangle_const_3 = 460

    parallel_seg_hex_1 = parallel_line_finder(hexagon_coeff_1, hexagon_const_1, bloat, "upper")
    parallel_seg_hex_2 = parallel_line_finder(hexagon_coeff_2, hexagon_const_2, bloat, "upper")
    parallel_seg_hex_3 = parallel_line_finder(hexagon_coeff_3, hexagon_const_3, bloat, "lower")
    parallel_seg_hex_4 = parallel_line_finder(hexagon_coeff_4, hexagon_const_4, bloat, "lower")

    parallel_seg_tri_1 = parallel_line_finder(triangle_coeff_1, triangle_const_1, bloat, "upper")
    parallel_seg_tri_2 = parallel_line_finder(triangle_coeff_2, triangle_const_2, bloat, "lower")
    if checker == 1:
        return parallel_seg_hex_1, parallel_seg_hex_2, parallel_seg_hex_3, parallel_seg_hex_4, parallel_seg_tri_1, parallel_seg_tri_2


    intersect_hex_1 = intersect_detect(hexagon_coeff_1, hexagon_coeff_2, parallel_seg_hex_1, parallel_seg_hex_2, 1, 1)

    intersect_hex_2 = intersect_detect(hexagon_coeff_1, -1, parallel_seg_hex_1, hexagon_const_6+bloat, 1, 0)
    intersect_hex_3 = intersect_detect(hexagon_coeff_4, -1, parallel_seg_hex_4, hexagon_const_6+bloat, 1, 0)

    intersect_hex_4 = intersect_detect(hexagon_coeff_3, hexagon_coeff_4, parallel_seg_hex_3, parallel_seg_hex_4, 1, 1)

    intersect_hex_5 = intersect_detect(hexagon_coeff_3, -1, parallel_seg_hex_3, hexagon_const_5-bloat, 1, 0)
    intersect_hex_6 = intersect_detect(hexagon_coeff_2, -1, parallel_seg_hex_2, hexagon_const_5-bloat, 1, 0)

    intersect_tri_1 = intersect_detect(triangle_coeff_1, triangle_coeff_2, parallel_seg_tri_1, parallel_seg_tri_2, 1, 1)
    intersect_tri_2 = intersect_detect(triangle_coeff_1, 0, parallel_seg_tri_1, 225 + bloat, 1, 1)

    intersect_tri_3 = [triangle_const_3 - bloat, 225 + bloat]
    intersect_tri_4 = [triangle_const_3 - bloat, 25 - bloat]
    
    intersect_tri_5 = intersect_detect(triangle_coeff_2, 0, parallel_seg_tri_2, 25 - bloat, 1, 1)

    return intersect_hex_1, intersect_hex_2, intersect_hex_3, intersect_hex_4, intersect_hex_5, intersect_hex_6, intersect_tri_1, intersect_tri_2, intersect_tri_3, intersect_tri_4, intersect_tri_5
      

def Branching_lines(display, col_1, col_3, begin, end, rad_conv):

    pg.draw.line(display, col_1, begin, end, 1)

    orientation = math.degrees(math.atan2(begin[1] - end[1], end[0] - begin[0])) + 90

    pg.draw.polygon(display, col_3, ((end[0]+rad_conv*math.sin(math.radians(orientation)), end[1]+rad_conv*math.cos(math.radians(orientation))),
                                     
                                           (end[0]+rad_conv*math.sin(math.radians(orientation-120)),
                                            end[1]+rad_conv*math.cos(math.radians(orientation-120))),

                                           (end[0]+rad_conv*math.sin(math.radians(orientation+120)), end[1]+rad_conv*math.cos(math.radians(orientation+120)))))




def game(bloat, hex_c1, hex_c2, hex_c3, hex_c4, hex_c5, hex_c6, tri_c1, tri_c2, tri_c3, tri_c4, tri_c5, visited, optimal_path, path):
    pg.init()
    size = [600, 250]
    display_view = pg.display.set_mode(size)
    pg.display.set_caption("Visualization")
    #video = vidmaker.Video("Explore.mp4", late_export=True)
    clock = pg.time.Clock()
    running = True

    rect_l_clr_1, rect_l_clr_11  = change_points_rect([100-bloat, 0], 250, 100+bloat)
    rect_l_wclr_2, rect_l_wclr_22 = change_points_rect([100, 0], 250, 100)
    rect_u_clr_3, rect_u_clr_33 = change_points_rect([100-bloat, 150-bloat], 250, 100+bloat)
    rect_u_clr_4, rect_u_clr_44 = change_points_rect([100, 150], 250, 100)

    tri_pt_1, tri_pt_11 = change_points(tri_c1, 250)
    tri_pt_2, tri_pt_22 = change_points(tri_c2, 250)
    tri_pt_3, tri_pt_33 = change_points(tri_c3, 250)
    tri_pt_4, tri_pt_44 = change_points(tri_c4, 250)
    tri_pt_5, tri_pt_55 = change_points(tri_c5, 250)

    tri_const_1, tri_const_11 = change_points([460, 25], 250)
    tri_const_2, tri_const_22 = change_points([460, 225], 250)
    tri_const_3, tri_const_33 = change_points([510, 125], 250)

    hex_const_1, hex_const_11 = change_points(hex_c1, 250)
    hex_const_2, hex_const_22 = change_points(hex_c2, 250)
    hex_const_3, hex_const_33 = change_points(hex_c3, 250)

    hex_const_4, hex_const_44 = change_points(hex_c4, 250)
    hex_const_5, hex_const_55 = change_points(hex_c5, 250)
    hex_const_6, hex_const_66 = change_points(hex_c6, 250)

    while running:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                running = False
        pg.draw.rect(display_view, "blue", [rect_l_clr_1, rect_l_clr_11, 50+2*bloat, 100+bloat], 0)
        pg.draw.rect(display_view, "red", [rect_l_wclr_2, rect_l_wclr_22, 50, 100], 0)
        pg.draw.rect(display_view, "blue", [rect_u_clr_3, rect_u_clr_33, 50+2*bloat, 100+bloat], 0)
        pg.draw.rect(display_view, "red", [rect_u_clr_4, rect_u_clr_44, 50, 100], 0)
        pg.draw.rect(display_view, "blue", [0, 0, bloat, 250], 0)
        pg.draw.rect(display_view, "blue", [0, 0, 600, bloat], 0)
        pg.draw.rect(display_view, "blue", [0, 250-bloat, 600, bloat], 0)
        pg.draw.rect(display_view, "blue", [600-bloat, 0, bloat, 250], 0)
        pg.draw.polygon(display_view, "blue", ([tri_pt_1, tri_pt_11], [tri_pt_2, tri_pt_22], [
                            tri_pt_3, tri_pt_33], [tri_pt_4, tri_pt_44], [tri_pt_5, tri_pt_55]), 0)
        pg.draw.polygon(
            display_view, "red", [[tri_const_1, tri_const_11], [tri_const_2, tri_const_22], [tri_const_3, tri_const_33]], 0)
        pg.draw.polygon(display_view, "blue", [[hex_const_1, hex_const_11], [hex_const_2, hex_const_22], [
                            hex_const_3, hex_const_33], [hex_const_4, hex_const_44], [hex_const_5, hex_const_55], [hex_const_6, hex_const_66]], 0)
        pg.draw.polygon(display_view, "red", ((
            235, 87.5), (300, 50), (365, 87.5), (365, 162.5), (300, 200), (235, 162.5)))
        for l in range(len(visited)-2):
            transition = visited[l]
            for i in range (len (path)):
                if path[i][2]==transition:
                    mat = path[i][1]
            transition = change_points(transition, 250)
            mat = change_points(mat, 250)
            #video.update(pg.surfarray.pixels3d(
            #display_view).swapaxes(0, 1), inverted=False)
            Branching_lines(display_view, "white", (127, 255, 212),
                  [transition[0], transition[1]], [mat[0], mat[1]], 0.5)
            pg.display.flip()
            clock.tick(300)

        for i in optimal_path:
            pg.draw.circle(display_view, (0, 255, 255), change_points(i, 250), radius-3)
            #video.update(pg.surfarray.pixels3d(
            #display_view).swapaxes(0, 1), inverted=False)
            pg.display.flip()
            clock.tick(20)
        running = False
    pg.display.flip()
    pg.time.wait(3000)
    pg.quit()

def orientation_block(bloat, hex_c1, hex_c2, hex_c3, hex_c4, tri_c1, tri_c2):
    occupied_space = OrderedSet()
    for x in np.arange(0, 601, 0.5):
        for y in np.arange(0, 251, 0.5):
            if (100 - bloat <=x<=150 + bloat) and (0<=y<=105):
                occupied_space.add((x, y))

            if (105 - bloat<=x<=145 + bloat) and (0<=y<=100):
                occupied_space.add((x, y))

            if (100 - bloat<=x<=150 + bloat) and (150 - bloat<=y<=245 + bloat):
                occupied_space.add((x, y))

            if (105 - bloat<=x<= 145 + bloat) and (155 - bloat<=y<=245 + bloat):
                occupied_space.add((x, y))    

            if (y+(2*x)-tri_c1) <= 0 and (y-(2*x)-tri_c2) >= 0 and ((460 - bloat)<=x) and ((25 - bloat)<= y and y <= (225 + bloat)):
                occupied_space.add((x, y))

            if (y-(15/26)*x - hex_c2) <= 0 and (y+(15/26)*x - hex_c1) <= 0 and (y-(15/26)*x - hex_c4) >= 0 and (y+(15/26)*x-hex_c3) >= 0 and ((235-bloat)<=x<=(365+bloat)):
                occupied_space.add((x, y))
                
            if (x <= 0 + bloat) or (y <= 0 + bloat) or (x >= 600 - bloat) or (y >= 250 - bloat):
                occupied_space.add((x, y))
    return occupied_space

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
            table.append([cost,node,new_node,0])
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
            table.append([cost,node,new_node,-2])
            open_list.put((cost,new_c2g,new_c2c,new_node))
            if dist(new_node,goal)<1.5:
                check_reach=True

def minus30_action(node,visited,stepsize,c2c,goal):
    global check_reach
    new_x=node[0]+stepsize*math.cos(np.deg2rad(node[2]-30))
    new_y=node[1]+stepsize*math.sin(np.deg2rad(node[2]-30))
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
            table.append([cost,node,new_node,-1])
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
            table.append([cost,node,new_node,1])
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
            table.append([cost,node,new_node,2])
            open_list.put((cost,new_c2g,new_c2c,new_node))
            if dist(new_node,goal)<1.5:
                check_reach=True


cleareance=int(input("input robot clearance "))
radius=int(input("input robot radius "))
while(True):
    stepsize=int(input("input stepsize "))
    if stepsize>=1 and stepsize<=10:
        break

parallel_seg_hex_1, parallel_seg_hex_2, parallel_seg_hex_3, parallel_seg_hex_4, parallel_seg_tri_1, parallel_seg_tri_2 = clearance((cleareance+radius), 1)
obstacle_space = orientation_block((cleareance + radius), parallel_seg_hex_1, parallel_seg_hex_2, parallel_seg_hex_3, parallel_seg_hex_4, parallel_seg_tri_1, parallel_seg_tri_2)
intersect_hex_1, intersect_hex_2, intersect_hex_3, intersect_hex_4, intersect_hex_5, intersect_hex_6, intersect_tri_1, intersect_tri_2, intersect_tri_3, intersect_tri_4, intersect_tri_5 = clearance(cleareance, 0)


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

open_list.put((dist(start,goal),dist(start,goal),0,start))
closed_list=np.zeros((1200,500,12),dtype=int)
closed_list_vis=OrderedSet()
new_goal=(0,0,0)
got_goal=0
back_track=[]
table.append([0,start,start,0.001])
while(open_list.empty()==False):
    current_node=open_list.get()
    now_node=current_node[3]
    if closed_list[int(now_node[0]*2)][int(now_node[1]*2)][int(now_node[2]/30)]!=1:
        closed_list[int(now_node[0]*2)][int(now_node[1]*2)][int(now_node[2]/30)]=1
        closed_list_vis.add((now_node[0],now_node[1],now_node[2]))
        if dist(now_node,goal)>1.5:
            if check_reach==False and now_node[0]>0 and now_node[0]+(stepsize*math.cos(np.deg2rad(now_node[2]+30)))<600 and now_node[1]>0 and now_node[1]+(stepsize*math.sin(np.deg2rad(now_node[2]+30)))<250:
                plus30_action(now_node,closed_list,stepsize,current_node[2],goal)
            if check_reach==False and now_node[0]>0 and now_node[0]+(stepsize*math.cos(np.deg2rad(now_node[2]+60)))<600 and now_node[1]>0 and now_node[1]+(stepsize*math.sin(np.deg2rad(now_node[2]+60)))<250:
                plus60_action(now_node,closed_list,stepsize,current_node[2],goal)
            if check_reach==False and now_node[0]>0 and now_node[0]+(stepsize*math.cos(np.deg2rad(now_node[2]+0)))<600 and now_node[1]>0 and now_node[1]+(stepsize*math.sin(np.deg2rad(now_node[2]+0)))<250:
                zero_action(now_node,closed_list,stepsize,current_node[2],goal)
            if check_reach==False and now_node[0]>0 and now_node[0]+(stepsize*math.cos(np.deg2rad(now_node[2]-60)))<600 and now_node[1]>0 and now_node[1]+(stepsize*math.sin(np.deg2rad(now_node[2]-60)))<250:
                minus60_action(now_node,closed_list,stepsize,current_node[2],goal)
            if check_reach==False and now_node[0]>0 and now_node[0]+(stepsize*math.cos(np.deg2rad(now_node[2]-30)))<600 and now_node[1]>0 and now_node[1]+(stepsize*math.sin(np.deg2rad(now_node[2]-30)))<250:
                minus30_action(now_node,closed_list,stepsize,current_node[2],goal)
        else:
            got_goal=1
            print('Goal reached')
            new_goal=(now_node[0],now_node[1],now_node[2])
            break
if got_goal==1:
    closed_list_vis.discard(start)
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
    game(cleareance,intersect_hex_1,intersect_hex_2,intersect_hex_3,intersect_hex_4,intersect_hex_5,intersect_hex_6,intersect_tri_1,intersect_tri_2,intersect_tri_3,intersect_tri_4,intersect_tri_5,closed_list_vis,back_track,table)
else:
    print('No solution')