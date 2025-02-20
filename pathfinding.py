# -*- coding: utf-8 -*-
"""
Created on Sat May 21 12:51:58 2022

@author: mohaa
"""
from PIL import ImageDraw, Image
import numpy as np

class cell(object):
    def __init__(self, filename=None, search=None, heuristic=None):
        with open(filename) as f:
            maze = f.readlines()
        
        self.cost = {'_': 1, 'f': 4, 'M': 10}
        self.w = 60
        self.h = 40
        self.start = (16, 18) # start point
        self.goal = (47, 20) # goal 
        self.map = maze[0:]        
        self.search = getattr(self, str(search))
        self.heuristic = getattr(self, str(heuristic))
        self.closedlist = set()
        self.path = dict()
        self.old_cost = dict()
        self.new_cost = dict()        
        self.index = 0
        self.current = self.search()
        pass

    def method(self):
        self.old_cost[self.start] = 0
        self.new_cost[self.start] = self.heuristic(self.start)
        self.path[self.start] = None
        self.add(self.start)# add start to openlist  
        while (len(self.openlist) > 0): # excute if openlist not empty
            parent = self.take()
            if self.found(parent): # check if parent is goal state
                return parent
            self.closedlist.add(parent) # add parent to closedlist
            added_neighbors = self.expand(parent)
            for child in added_neighbors: 
                if child not in self.closedlist: # add cost if child not in closedlist
                    cost_child = self.old_cost[parent] + self.added_cost(child)
                    if child not in self.openlist or cost_child < self.old_cost[child]:
                        self.path[child] = parent
                        self.old_cost[child] = cost_child
                        self.new_cost[child] = cost_child + self.heuristic(child)
                        if child not in self.openlist:
                            self.add(child)    

    
    
    def costtoreach(self, state):
        price = self.cost[self.map[state[1]][state[0]]] # cost of the move
        return price
  
    def breadth_first(self):
        self.openlist = list()
        self.take = self.take_not_sorted
        self.add = self.add_not_sorted
        self.added_cost = self.none
        return self.method()
    
    def lowest_cost(self):
        self.openlist = set() # sorting is needed 
        self.take = self.take_sorted
        self.add = self.add_sorted # if element already exists don't add
        self.sort = self.low_cost
        self.added_cost = self.costtoreach
        return self.method()
    
    
    def greedy_best_first(self):
        self.openlist = set()  # sorting is needed
        self.take = self.take_sorted
        self.add = self.add_sorted
        self.sort = self.sort_estimated # sort by estimated cost
        self.added_cost = self.none
        return self.method()
    
    def astar(self):
        self.openlist = set() # sorting is needed
        self.take = self.take_sorted
        self.add = self.add_sorted
        self.sort = self.sort_estimated_total # sort by estimated and true cost
        self.added_cost = self.costtoreach
        return self.method()
    
    def take_not_sorted(self):
        self.index += 1
        return self.openlist[self.index - 1]
    
    def take_sorted(self):
        sorted_list = self.sort()
        i = 0        
        for i in range(len(sorted_list) - 1):
            if sorted_list[i] not in self.closedlist:
                break
        popped = sorted_list[i]
        self.openlist.remove(popped)
        return popped
    
    def add_not_sorted(self, state):
        self.openlist.append(state)
        
    def add_sorted(self, state):
        self.openlist.add(state)
        
        
    def low_cost(self):
        return sorted(self.new_cost, key=lambda state: self.old_cost[state])
    
    def found(self, state):
        return self.goal == state
    
    def expand(self, state):
        expanded = list()
        x, y = state
        for neighbor in ((x+1, y), (x-1, y), (x, y+1), (x, y-1)):
            if self.valid(neighbor):  # make sure it's not water or out of bounds
                expanded.append(neighbor) # add neighbor to openlist
        return expanded
    
    def valid(self, state):
        x, y = state
        return x >= 0 and y >= 0 and x < self.w and y < self.h and self.map[y][x] != '~' and self.map[y][x] != 'x'
    
    
    def sort_estimated(self):
        sorting = sorted(self.new_cost, key=lambda state: self.heuristic(state))
        return sorting
    
    def sort_estimated_total(self):
        sorting = sorted(self.new_cost, key=lambda state: self.old_cost[state] + self.heuristic(state))
        return sorting
    
    
    def result(self):
        total_cost = 0
        step = [self.current]
        state = self.path[self.current]
        while state is not None:
            step.append(state)
            total_cost += self.costtoreach(state)
            state = self.path[state]
        step.reverse()
        return step, total_cost 



    @staticmethod
    def none(state):
        return 0
    
    def manhattan(self, state):
        x1, y1 = self.goal
        x2, y2 = state
        z = abs(x1 - x2) + abs(y1 - y2)
        return z
    
    def euclidean(self, state):
        x1, y1 = self.goal
        x2, y2 = state
        z = np.sqrt(((x1 - x2) ** 2 + (y1 - y2) ** 2))
        return z
    
    
    
def main():
    
    chosen_color = {'_': [255, 255, 255], # black road
             'f': [85, 100, 45], # green forest
             'M': [128, 128, 128], # grey mountain
             '~': [0, 0, 128], # blue water
             'x': [0, 0, 0]} #black borders
    
    search_types = ['breadth_first', 'none',
             'lowest_cost', 'none',
             'greedy_best_first', 'euclidean',
             'greedy_best_first', 'manhattan',
             'astar', 'euclidean',
             'astar', 'manhattan']

    
    for x in range(0, len(search_types), 2):
        type_name = search_types[x]
        if search_types[x + 1] is not 'none':
            type_name += '_' + search_types[x + 1]
    
        smap = cell('map.txt', search_types[x], search_types[x+1]) # saerched map 
        path, costs = smap.result()
        print('Cost: ', costs)
        print('Open List: ', len(smap.openlist))
        print('Closed List: ', len(smap.closedlist))
        print('Lenght: ', len(path)-1)
        
                
        height = smap.h * 10 + 1
        width = smap.w * 10 + 1
        delta = 21
        new_image = Image.new('RGB', (width, height + delta), color='black')
        
        pic = ImageDraw.Draw(new_image)
        for i in range(smap.w + 1):
            ci = i * 10
            for j in range(smap.h + 1):
                cj = j *10.5
                if j < smap.h and i < smap.w:
                    color = chosen_color[smap.map[j][i]]
                    outline = ((ci + 3, cj + 3), (ci + 7, cj + 7))
                    solid = ((ci + 4, cj + 4), (ci + 7, cj + 7))
                    pic.rectangle(((ci + 1, cj + 1), (ci + 9, cj + 9)), tuple(color))
                    if (i, j) in path:
                        pic.ellipse(solid, 'red')
                    if (i, j) in smap.closedlist:
                        pic.ellipse(outline, None, 'red')
                    elif (i, j) in smap.openlist:
                        pic.ellipse(outline, None, 'black')
                    if (i, j) == smap.start:
                        pic.ellipse(outline, None, 'black')
                    elif (i, j) == smap.goal:
                        pic.ellipse(outline, None, 'green')
    
    
        print('Picture: ' + type_name + '.png')

        new_image.save(type_name + '.png')


main()  