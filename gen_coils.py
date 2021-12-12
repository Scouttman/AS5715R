#!/usr/bin/env python3
from __future__ import print_function, division
import math
import pcbnew
from pcbnew import TEXTE_MODULE, TEXTE_PCB, EDA_TEXT
import numpy as np

# import os
# os.chdir("my_stuff/BLDC/encoder/python_hello")
# os.chdir("C:\\Users\\scott.fraser\\Documents\\Algos\\motor\\encoder\\inductive_encoder")
# import gen_coils
# reload(gen_coils)
""" Script to generate a kicad pcb heater block of a given size """
"""
cd my_stuff/old/encoder
import gen_coils
import importlib
def gen():
    importlib.reload(gen_coils)
"""

#https://read01.com/8ao6DkO.html
#https://github.com/mattvenn/kicad/blob/master/python_api/test.py#L74
#https://kicad.mmccoo.com/2017/02/
#https://kicad.mmccoo.com/2017/02/01/the-basics-of-scripting-in-pcbnew/

# most queries start with a board
SCALE = 1000000
track_thickness = 0.2

def get_layertable():
    layertable = {}
    board = pcbnew.GetBoard()

    numlayers = pcbnew.PCB_LAYER_ID_COUNT
    for i in range(numlayers):
        layertable[board.GetLayerName(i)] = i
        #print("{} {}".format(i, board.GetLayerName(i)))
    return layertable

def get_nettable():
    board = pcbnew.GetBoard()
    nets = board.GetNetsByName()
    # for netname, net in nets.items():
    #     print("netcode {}, name{}".format(net.GetNet(), netname))
    return nets

def add_via(x,y,netCode = None):
    board = pcbnew.GetBoard()
    layertable = get_layertable()
    nets = board.GetNetsByName()
    newvia = pcbnew.VIA(board)
    board.Add(newvia)
    newvia.SetLayerPair(layertable["F.Cu"], layertable["B.Cu"])
    newvia.SetPosition(pcbnew.wxPoint(x,y))
    newvia.SetViaType(pcbnew.VIA_THROUGH)
    newvia.SetWidth(int(SCALE*0.7))
    if(netCode is not None):
        net = nets.find(netCode).value()[1]
        newvia.SetNet(net)

def add_tracks():
    layertable = get_layertable()
    board = pcbnew.GetBoard()
    for i in range(100):
        track = pcbnew.TRACK(board)
        track.SetStart(pcbnew.wxPoint(0, SCALE*i))
        track.SetEnd(pcbnew.wxPoint(SCALE*200, SCALE*i))
        track.SetWidth(int(SCALE*0.7))
        track.SetLayer(layertable["B.Cu"])
        track.SetNet()
        board.Add(track)
    pcbnew.Refresh()

def add_circle(res, main_diam, var_diam, waves, offset, layer, rotate, skip=0, netCode = None, netCodeEnd = None, vias = True):
    layertable = get_layertable()
    board = pcbnew.GetBoard()
    nets = board.GetNetsByName()
    first = True
    for i in range(rotate, res-skip+rotate):
        theta = math.pi*i/(res/2)
        if(theta>math.pi):
            theta = -2*math.pi+theta
        
        track = pcbnew.TRACK(board)
        A = math.sin(math.pi*i/(res/2))
        B = math.cos(math.pi*i/(res/2))
        C = math.sin(math.pi*(i+1)/(res/2))
        D = math.cos(math.pi*(i+1)/(res/2))
        diam_1 = main_diam+var_diam*math.sin(math.pi*i/(res/(2*waves))+offset)
        diam_2 = main_diam+var_diam*math.sin(math.pi*(i+1)/(res/(2*waves))+offset)
        track.SetStart(pcbnew.wxPoint(A*diam_1,B*diam_1))
        track.SetEnd(pcbnew.wxPoint(C*diam_2,D*diam_2))
        track.SetWidth(int(SCALE*track_thickness))
        track.SetLayer(layertable[layer])
        if(netCode is not None):
            net = nets.find(netCode).value()[1]
            track.SetNet(net)
        board.Add(track)
        if(first and vias):
            add_via(A*diam_1,B*diam_1, netCode)
            first = False
    if(vias):
        add_via(C*diam_2, D*diam_2, netCodeEnd)  
    pcbnew.Refresh()

def add_wave(res, main_diam, var_diam, waves, offset, end = 0, turn_around=0, skip=0, netCode = None, end_point = None, netCodeEnd = None, netCodeStart = None, art = False):
    layertable = get_layertable()
    board = pcbnew.GetBoard()
    nets = board.GetNetsByName()
    first = True
    thetas = np.linspace(0, 2*math.pi, res)
    thetas_wave  = (waves*thetas+offset) # 
    diam_wave = np.sin(thetas_wave)*var_diam
    diam = diam_wave+main_diam
    circle_sin = np.sin(thetas)
    circle_cos = np.cos(thetas)
    layer = np.sin(thetas_wave+math.pi/2)>0
    layer_change = np.diff(layer) != 0
    if(end_point is not None):
        start_end = np.argwhere(layer_change)
        if(start_end.size < 2):
            start_end = [start_end[0], thetas.size-1]
        layer_change[start_end[end_point,0]] = False # remove first via


    space = 0.03
    inflextion_pt = [turn_around-space, turn_around+space]
    cross_over = (inflextion_pt[0] < thetas) & (thetas <  inflextion_pt[1])
    netCode = netCodeEnd

    x = circle_sin*diam
    y = circle_cos*diam
    for i in range(res-1):
        track = pcbnew.TRACK(board)
        track.SetStart(pcbnew.wxPoint(x[i],y[i]))
        track.SetEnd(pcbnew.wxPoint(x[i+1],y[i+1]))
        track.SetWidth(int(SCALE*track_thickness))
        if(not art):
            if(layer[i]):
                track.SetLayer(layertable["In1.Cu"])
            else:
                track.SetLayer(layertable["In2.Cu"])
            if(layer_change[i]):
                add_via(x[i+1],y[i+1], netCode)

            if(end_point is not None):
                if(i == start_end[0]):
                    netCode = netCodeStart
                if(i == start_end[-1]):
                    netCode = netCodeEnd

            if(netCode is not None):
                net = nets.find(netCode).value()[1]
                track.SetNet(net)
            if(not cross_over[i]):
                board.Add(track)
            # gaps for turn around
            elif((not cross_over[i+1]) & layer[i+1]):
                netCode = None
                add_via(x[i+1],y[i+1], netCode)
            elif((not cross_over[i-1]) & layer[i-1]):
                netCode = None
                add_via(x[i],y[i], netCode)
        else:
            if(not layer[i]):
                # track.SetLayer(layertable["F.SilkS"])
                # board.Add(track)
                seg1 = pcbnew.DRAWSEGMENT(board)
                board.Add(seg1)
                seg1.SetStart(pcbnew.wxPoint(x[i],y[i]))
                seg1.SetEnd(pcbnew.wxPoint(x[i+1],y[i+1]))
                seg1.SetLayer(layertable["B.SilkS"])
        # if(first and vias):
        #     add_via(x[i],y[i], netCode)
        #     first = False
    # if(vias):
    #     add_via(x[i+1],y[i+1], netCodeEnd)  
    pcbnew.Refresh()

def add_circle_edge(res, diam, skip_start, skip_end):
    layertable = get_layertable()
    board = pcbnew.GetBoard()
    edgecut = layertable['Edge.Cuts']
    skip_pos = [-100,-100]

    # draw the circle
    for i in range(res):
        theta = math.pi*i/(res/2)
        theta_2 = math.pi*(i+1)/(res/2)
        if(theta>math.pi):
            theta = -2*math.pi+theta
        #edge cuts
        if(theta > skip_start):
            if(theta < skip_end):
                continue

        if(skip_pos[0]==-100):
            skip_pos[0] = theta
        skip_pos[1] = theta_2

        A = math.sin(theta)
        B = math.cos(theta)
        C = math.sin(theta_2)
        D = math.cos(theta_2)

        seg1 = pcbnew.DRAWSEGMENT(board)
        board.Add(seg1)
        seg1.SetStart(pcbnew.wxPoint(A*diam, B*diam))
        seg1.SetEnd(pcbnew.wxPoint(C*diam, D*diam))
        seg1.SetLayer(edgecut)

    for theta in skip_pos:
        if(theta>-100 and theta !=0 and theta < 6.28):
            A = math.sin(theta)
            B = math.cos(theta)
            seg1 = pcbnew.DRAWSEGMENT(board)
            board.Add(seg1)
            seg1.SetStart(pcbnew.wxPoint(A*diam, B*diam))
            seg1.SetEnd(pcbnew.wxPoint(A*diam, B*diam+10*SCALE))
            seg1.SetLayer(edgecut)

def add_coil(res,main_diam,loops,gap,layer,offset = 0, netCode = None, region = 1):
    layertable = get_layertable()
    board = pcbnew.GetBoard()
    start = True
    nets = board.GetNetsByName()
    for i in range(offset, res*loops + offset-region):
        track = pcbnew.TRACK(board)
        A = math.sin(math.pi*i/(res/2))
        B = math.cos(math.pi*i/(res/2))
        C = math.sin(math.pi*(i+1)/(res/2))
        D = math.cos(math.pi*(i+1)/(res/2))
        shift_1 = ((i-offset)//res)*gap
        shift_2 = shift_1
        if((i-offset)%res-res+region+1>0):
            shift_1+=(((i-offset)%res-res+region)/region)*gap
            shift_2+=(((i-offset)%res-res+region+1)/region)*gap
        diam_1 = main_diam+shift_1
        diam_2 = main_diam+shift_2
        track.SetStart(pcbnew.wxPoint(A*diam_1, B*diam_1))
        track.SetEnd(pcbnew.wxPoint(C*diam_2, D*diam_2))
        track.SetWidth(int(SCALE*track_thickness))
        track.SetLayer(layertable[layer])
        if(netCode is not None):
            net = nets.find(netCode).value()[1]
            track.SetNet(net)
        board.Add(track)
        if(start):
            start = False
            add_via(A*diam_1,B*diam_1-gap//2,netCode)
    # add_via(C*diam_2, D*diam_2+gap//2, netCode)
    pcbnew.Refresh()



diam_internal = 13/2
diam_external = 33/2
diam_coils_rx = (diam_internal+diam_external)/2
gap_tx_rx = 0.8
edge_gap = 0.2
via_space = 0.2
width_coils_tx = 3*0.4
width_coils_rx = (diam_external-diam_internal)/2-(edge_gap+via_space)
diam_coils_tx_in = diam_coils_rx-width_coils_rx-gap_tx_rx+via_space
diam_coils_tx_ex = diam_coils_rx+width_coils_rx+gap_tx_rx-via_space

# Outline 
# print("hi")
# add_circle_edge(100, diam_internal*SCALE, 0, 0)
# # add_circle_edge(100, diam_external*SCALE, -math.pi/8, math.pi/8)
# # Rx coilsint(SCALE*0.3/2)
# num_segs = 4
# add_wave(1000, diam_coils_rx*SCALE,width_coils_rx*SCALE, num_segs,           0, end = 0,turn_around=math.pi, end_point = 0, netCodeEnd="RXSB", netCodeStart="RXCB")
# add_wave(1000, diam_coils_rx*SCALE,width_coils_rx*SCALE, num_segs, math.pi/2, end = math.pi*(0.5/num_segs), 
#             turn_around=math.pi*(1-0.5/num_segs), end_point = [0,-1], netCodeEnd="RXCA", netCodeStart="RXSA")
# add_wave(1000, diam_coils_rx*SCALE,width_coils_rx*SCALE, num_segs, math.pi, end = 0, turn_around=math.pi)
# add_wave(1000, diam_coils_rx*SCALE,width_coils_rx*SCALE, num_segs, 3*math.pi/2, end = math.pi*(0.5/num_segs), turn_around=math.pi*(1-0.5/num_segs))

# art
for i in range(4):
    add_wave(1000, diam_coils_rx*SCALE,width_coils_rx*SCALE, num_segs, i*math.pi/2, art=True)

# # tx coils
# #add_coil(50,diam_coils_tx_in*SCALE,3,-0.4*SCALE,"F.Cu", 1, netCode="TXB") # inner
# add_coil(50,diam_coils_tx_ex*SCALE,3, 0.4*SCALE,"F.Cu", 100-1, netCode="TXB") # outer
# add_coil(50,diam_coils_tx_ex*SCALE,3, 0.4*SCALE,"B.Cu", 100-1, netCode="TXA") # outer

# board = pcbnew.GetBoard()
# nets = board.GetNetsByName()
# for netname, net in nets.items():
#     print("netcode {}, name {}".format(net.GetNet(), netname))


"""
import os
os.chdir("C:\\Users\\scott.fraser\\Documents\\Algos\\motor\\encoder\\inductive_encoder")
import gen_coils
reload(gen_coils)
"""
