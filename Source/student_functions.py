import numpy as np
from collections import deque
import queue
def BFS(matrix, start, end):
    
    """
    BFS algorithm:
    Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes, each key is a visited node,
        each value is the adjacent node visited before it.
    path: list
        Founded path
    """
   
    # TODO: 
   
    path=[] # lưu kết quả đường đi từ điểm bắt đầu đến đích
    visited={} #lưu các đỉnh đã thắm
    q = queue.Queue() #tạo danh sách hàng đợi lưu các đỉnh sẽ thăm
    q.put((start,None)) #thêm điểm bắt đầu vào danh sách hàng đợi và đỉnh đã thăm nó
    visited={start: None} # thêm điểm bắt đầu vào đỉnh đã thăm
    visit={start: None}# đánh dấu đã được duyệt trong danh sách hàng đợi

    while q:# vòng lặp kết thúc khi danh sách hàng đợi rộng
        current_node,front_node = q.get() #lấy đỉnh nằm đầu tiên của hàng đợi và đỉnh đã thăm nó
        visited[current_node] = front_node# lưu đỉnh đã thăm 

        if current_node == end:# nếu như đỉnh đang duyệt giống với đich thì dừng vòng lặp
            break

        for neighbor in range(len(matrix[current_node]) - 1, -1, -1):# duyệt các đỉnh hàng xóm của đỉnh hiện tại
            if matrix[current_node][neighbor] != 0 and neighbor not in visit:# điều kiện để thêm vào danh sách hàng đợi
                q.put((neighbor,current_node))#thêm vào cách đỉnh cạnh nó vào danh sách hàng đợi
                visit[neighbor] = current_node  # Đánh các đỉnh chuẩn bị thăm
    # nêu tìm thấy đỉnh thì lưu đường đi vào path
    if end in visited:
        current_node = end
        while current_node is not None:
            path.append(current_node)
            current_node = visited[current_node]
        path.reverse()
    print("")    
    print("Cac dinh da tham" ,visited)
    print("duong di ",start, " -> ", end,": ",path)
    return visited, path


def DFS(matrix, start, end):
    """
    DFS algorithm
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited 
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """

    # TODO: 
    
    path=[]
    visited={start:None}
    stack = [(start,None)]  # Ngăn xếp DFS, lưu đỉnh và đỉnh cha của nó

    while (stack):
        current_node,previous = stack.pop()# Lấy đỉnh cuối cùng trong ngăn xếp
        visited[current_node]=previous   # Lưu đỉnh cha của current_node 

        if current_node== end: # Nếu đã đến đích, thoát khỏi vòng lặp
            break
         # Duyệt các đỉnh kề của current_node
        for neighbor in range(len(matrix[current_node])):
            if(matrix[current_node][neighbor] != 0 and neighbor not in visited):
                stack.append((neighbor,current_node))# Thêm đỉnh kề vào ngăn xếp
     # Nếu tìm thấy đỉnh đích, xây dựng đường đi
    if end in visited:
        current_node = end
        while current_node is not None:
            path.append(current_node)
            current_node = visited[current_node]
        path.reverse() # Đảo ngược để có đường đi từ start đến end
    print("")    
    print("Cac dinh da tham" ,visited)
    print("duong di ",start, " -> ", end,": ",path)
    return visited, path

def UCS(matrix, start, end):
    """
    Uniform Cost Search algorithm
     Parameters:visited
    ---------------------------
    matrix: np array
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
   # TODO:  
    path = []
    visited = {start: None} 
    priority_queue = queue.PriorityQueue()
    priority_queue.put((0, (start,None)))  # Hàng đợi lưu (chi phí, (đỉnh, đỉnh cha))
  
    while not priority_queue.empty():

        cost, (current_node,front_node) = priority_queue.get()# Lấy đỉnh có chi phí nhỏ nhất
        visited[current_node] = front_node  # Lưu đỉnh cha của current_node 


        if current_node == end: # Kiểm tra nếu đã đến đỉnh đích
            break
        
         # Duyệt qua các đỉnh kề của current_node
        for neighbor in range(len(matrix[current_node]) - 1, -1, -1):
            if matrix[current_node][neighbor] != 0 and neighbor not in visited:# kiểm tra đỉnh đã thăm và có đường đi tới đỉnh không.
                new_cost = cost + matrix[current_node][neighbor]# tính phí đi từ điểm bắt đầu đến đỉnh kề.
                priority_queue.put((new_cost, (neighbor,current_node)))# thêm các định kề vào danh sách hàng đợi
    # Nếu tìm thấy đỉnh đích, xây dựng đường đi
    if end in visited:
        current_node = end
        while current_node is not None:
            path.append(current_node)
            current_node = visited[current_node]
        path.reverse() # Đảo ngược để có đường đi từ start đến end

    print("")    
    print("Cac dinh da tham" ,visited)
    print("duong di ",start, " -> ", end,": ",path)
    return visited, path

def GBFS(matrix, start, end):
    """
    Greedy Best First Search algorithm 
    heuristic : edge weights
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
   
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:  
    path = []
    visited = {start: None}
    pq = queue.PriorityQueue()
    pq.put((0, (start,None)))# Hàng đợi lưu (trọng số của cạnh, (đỉnh, đỉnh cha))
    while not pq.empty():
        _, (current_node,front_node) = pq.get()# Lấy đỉnh có trọng số nhỏ nhất
        visited[current_node] = front_node #lưu đỉnh cha của node
        if current_node == end: #kiểm tra đã tới đich chưa
            break
        # duyệt các đỉnh kề của node
        for neighbor in range(len(matrix[current_node]) - 1, -1, -1):
            weight = matrix[current_node][neighbor] #lấy cân nặng từ node đên đỉnh con
            if weight and neighbor not in visited:  #kiểm tra đỉnh con đã thăm và có đường đi tới đỉnh con không
                if(neighbor == end):# kiểm tra đỉnh kề có phải đỉnh đích không
                    weight = 0
                pq.put((weight, (neighbor,current_node)))# thêm đỉnh kề vào danh sách hàng đợi
    # Nếu tìm thấy đỉnh đích, xây dựng đường đi
    if end in visited:
        current_node = end
        while current_node is not None:
            path.append(current_node)
            current_node = visited[current_node]
        path.reverse()# Đảo ngược để có đường đi từ start đến end
    print("")    
    print("Cac dinh da tham" ,visited)
    print("duong di ",start, " -> ", end,": ",path)
    return visited, path

def heuristic(star,end,pos): # tính heuristic
    x1,y1=pos[star]
    x2,y2=pos[end]
    return ((x1-x2)**2+ (y1-y2)**2)**0.5

def Astar(matrix, start, end, pos):
    """
    A* Search algorithm
    heuristic: eclid distance based positions parameter
     Parameters:
    ---------------------------
    matrix: np array UCS
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    pos: dictionary. keys are nodes, values are positions
        positions of graph nodes
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO: 
    path=[]
    visited={start: None}
    pq=queue.PriorityQueue()
    pq.put((heuristic(start,end,pos),(start,None)))# Hàng đợi lưu (f(x)= h(x) + g(x), (đỉnh, đỉnh cha)) 
                                                    # h(x) là hàm ước lượng khoảng cách từ điểm tới đích
                                                    # g(x) là chi phi đi từ đỉnh bắt đầu đến đỉnh
    while not pq.empty():
        cost,(current_node,front_node) = pq.get()# lấy đỉnh có f(x) nhỏ nhất có trong danh sách
        visited[current_node] =front_node # lưu nút cha của đỉnh.

        if current_node == end:#kiểm tra đỉnh có phải là đích không
            break
        # duyệt có đỉnh kề của đỉnh
        for neighbor in range(len(matrix[current_node]) - 1, -1, -1):
            # kiểm tra các đỉnh kề đã được thăm hay chưa có đường đi tới đỉnh kề không
            if matrix[current_node][neighbor]!=0 and neighbor not in visited:     
                #tính f(x) của đỉnh kề
                new_cost = cost-heuristic(current_node,end,pos)+matrix[current_node][neighbor]+heuristic(neighbor,end,pos)
                pq.put((new_cost,(neighbor,current_node))) # thêm đỉnh kề vào danh sách hàng đợi ưu tiên
                
     # Nếu tìm thấy đỉnh đích, xây dựng đường đi
    if end in visited:
        current_node = end
        while current_node is not None:
            path.append(current_node)
            current_node = visited[current_node]
        path.reverse()# Đảo ngược để có đường đi từ start đến end
    print("")    
    print("Cac dinh da tham" ,visited)
    print("duong di ",start, " -> ", end,": ",path)
    return visited, path

