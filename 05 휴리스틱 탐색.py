#!/usr/bin/env python
# coding: utf-8

# 휴리스틱 탐색 알로리즘을 이해하기 위한 자료. 여기 제공하는 코드는 GitHub aima-python의 코드를 기반으로 일부 수정한 것임.

# # 탐색을 통한 문제 해결 기반 구조
# 기반 구조들은 매번 재정의할 필요가 없으므로, 이들을 search_common.py에 옮겨 저장해뒀음. 앞으로는 이 모듈을 import해서 사용하면 됨.

# In[ ]:


# 탐색을 통한 문제 해결을 위해 필요한 기반 구조 import
from search_common import *


# # 휴리스틱 탐색 알고리즘 구현

# ## 최고 우선 탐색(best-first search)

# In[ ]:


def best_first_search(problem, f):
    """최고 우선 탐색: 평가함수 f(n)값이 가장 낮은 노드 n부터 탐색.
    어떤 평가함수를 사용할 것인지를 f에 명시해주어야 함.
    어떤 노드에서부터 목표까지의 비용 추정치(휴리스틱)로 지정해주면,
    결국 이 탐색은 탐욕적 최고 우선 탐색이 됨."""
    
    node = Node(problem.initial)
    frontier = PriorityQueue([node], key=f)
    reached = {problem.initial: node}
    while frontier:
        node = frontier.pop()
        if problem.is_goal(node.state):
            return node
        for child in expand(problem, node):
            s = child.state
            if s not in reached or child.path_cost < reached[s].path_cost:
                reached[s] = child
                frontier.add(child)
    return failure


def best_first_tree_search(problem, f):
    """도달 상태 테이블(reached) 없이 탐색을 수행하는 최고 우선 탐색 버전"""
    frontier = PriorityQueue([Node(problem.initial)], key=f)
    while frontier:
        node = frontier.pop()
        if problem.is_goal(node.state):
            return node
        for child in expand(problem, node):
            if not is_cycle(child):
                frontier.add(child)
    return failure


def is_cycle(node, k=30):
    "node가 길이 k이하인 싸이클을 형성하는가?"
    def find_cycle(ancestor, k):
        return (ancestor is not None and k > 0 and
                (ancestor.state == node.state or find_cycle(ancestor.parent, k - 1)))
    return find_cycle(node.parent, k)


def g(n): return n.path_cost


# ## 탐욕적 최고 우선 탐색

# In[ ]:


def greedy_bfs(problem, h=None):
    """h(n)이 가장 낮은 노드부터 탐색"""
    h = h or problem.h
    return best_first_search(problem, f=h)


# ## 균일 비용 탐색(uniform-cost search)

# In[ ]:


def uniform_cost_search(problem):
    """경로 비용(g)이 가장 낮은 노드부터 탐색"""
    return best_first_search(problem, f=g)


# ## A* 탐색

# In[ ]:


def astar_search(problem, h=None):
    """f(n) = g(n)+h(n)이 가장 낮은 노드 n부터 탐색.
    휴리스틱 함수를 h로 넘겨 주지 않으면 problem에 정의된 기본 h 함수가 사용됨."""
    h = h or problem.h
    return best_first_search(problem, f=lambda n: g(n) + h(n))


def astar_tree_search(problem, h=None):
    """도달 상태 테이블(reached) 없이 탐색을 수행하는 버전"""
    h = h or problem.h
    return best_first_tree_search(problem, f=lambda n: g(n) + h(n))


# ## 가중치 A* 탐색

# In[ ]:


def weighted_astar_search(problem, h=None, weight=1.4):
    """f(n) = g(n) + weight * h(n)에 따라 f값이 가장 낮은 노드부터 탐색"""
    h = h or problem.h
    return best_first_search(problem, f=lambda n: g(n) + weight * h(n))


# ## 너비/깊이 우선 탐색 재정의
# 최고 우선 탐색을 사용하여 너비/깊이 우선 탐색도 구현할 수 있음. 평가 함수 f를 깊이에 따라 정의하면 됨.

# In[ ]:


def breadth_first_bfs(problem):
    """f(n)=노드n의 깊이로 정의하여 best-first search 수행"""
    return best_first_search(problem, f=len)


def depth_first_bfs(problem):
    """f(n)=-(노드n의 깊이)로 정의하여 best-first search 수행"""

    return best_first_search(problem, f=lambda n: -len(n))


# # 문제 해결: 루마니아 여행

# ## 문제 정의 (지난 수업 내용)

# In[ ]:


class Map:
    """2차원 지도를 표현하는 그래프.
    Map(links, locations) 형태로 생성.
    - links: [(v1, v2)...] 또는 {(v1, v2): distance...} dict 구조 가능
    - locations: {v1: (x, y)} 형식으로 각 노드의 위치(좌표) 설정 가능
    - directed=False이면, 양방향 링크 추가. 즉, (v1, v2) 링크에 대해 (v2, v1) 링크 추가"""

    def __init__(self, links, locations=None, directed=False):
        if not hasattr(links, 'items'):
            links = {link: 1 for link in links}  # 거리 기본값을 1로 설정
        if not directed:
            for (v1, v2) in list(links):
                links[v2, v1] = links[v1, v2]
        self.distances = links
        self.neighbors = multimap(links)  # 인접 리스트
        self.locations = locations or defaultdict(lambda: (0, 0))

        
def multimap(pairs):
    """(key, val) 쌍들이 주어지면, {key: [val,...]} 형태의 dict 생성하여 리턴."""
    result = defaultdict(list)
    for key, val in pairs:
        result[key].append(val)
    return result


# In[ ]:


class RouteProblem(Problem):
    """지도(Map) 상의 위치 간의 이동 경로를 알아 내는 문제.
    RouteProblem(초기상태, 종료상태, map=Map(...)) 형식으로 문제 생성.
    상태: Map 그래프의 노드들(예: 'A' - 위치 이름이 'A'),
    행동: 목적지 상태들(예: 'A' - 위치 'A'로 이동하는 행동)"""
    
    def actions(self, state): 
        """state에 인접한 장소들"""
        return self.map.neighbors[state]
    
    def result(self, state, action):
        """지도 상에서 가능하다면, action 위치로 이동.
        불가능하다면, 위치(상태) 변화 없음. 즉, 위치(상태)는 그대로 state"""
        return action if action in self.map.neighbors[state] else state
    
    def action_cost(self, s, action, s1):
        """s에서 s1로 이동하는 비용: 거리"""
        return self.map.distances[s, s1]
    
    def h(self, node):
        "node의 상태와 목표 상태 사이의 직선 거리"
        locs = self.map.locations
        return straight_line_distance(locs[node.state], locs[self.goal])
    
    
def straight_line_distance(A, B):
    "두 점 사이의 직선 거리"
    return sum(abs(a - b)**2 for (a, b) in zip(A, B)) ** 0.5


# In[ ]:


# 루마니아 지도 정의
romania = Map(
    {('O', 'Z'):  71, ('O', 'S'): 151, ('A', 'Z'): 75, ('A', 'S'): 140, ('A', 'T'): 118, 
     ('L', 'T'): 111, ('L', 'M'):  70, ('D', 'M'): 75, ('C', 'D'): 120, ('C', 'R'): 146, 
     ('C', 'P'): 138, ('R', 'S'):  80, ('F', 'S'): 99, ('B', 'F'): 211, ('B', 'P'): 101, 
     ('B', 'G'):  90, ('B', 'U'):  85, ('H', 'U'): 98, ('E', 'H'):  86, ('U', 'V'): 142, 
     ('I', 'V'):  92, ('I', 'N'):  87, ('P', 'R'): 97},
    {'A': ( 76, 497), 'B': (400, 327), 'C': (246, 285), 'D': (160, 296), 'E': (558, 294), 
     'F': (285, 460), 'G': (368, 257), 'H': (548, 355), 'I': (488, 535), 'L': (162, 379),
     'M': (160, 343), 'N': (407, 561), 'O': (117, 580), 'P': (311, 372), 'R': (227, 412),
     'S': (187, 463), 'T': ( 83, 414), 'U': (471, 363), 'V': (535, 473), 'Z': (92, 539)})


# In[ ]:


# 루마니아 여행 문제 정의
r0 = RouteProblem('A', 'A', map=romania)
r1 = RouteProblem('A', 'B', map=romania)  # 초기상태: A, 목표상태: B
r2 = RouteProblem('N', 'L', map=romania)
r3 = RouteProblem('E', 'T', map=romania)
r4 = RouteProblem('O', 'M', map=romania)


# ## A* 탐색을 이용한 루마니아 여행 문제 해결

# In[ ]:


path_states(greedy_bfs(r1))


# In[ ]:


path_states(astar_search(r1))


# In[ ]:


path_states(weighted_astar_search(r1))


# In[ ]:


for problem in [r0, r1, r2, r3, r4]:
    print(path_states(astar_search(problem)))


# # 문제 해결: 8-퍼즐

# ## 문제 정의

# 3x3 보드 판에서 숫자 타일을 빈 공간으로 움직여 목표 상태의 숫자 판 배열로 만드는 게임.
# 예: (0은 빈 공간을 의미함)
# 
#           초기 상태                           목표 상태
#           | 7 | 2 | 4 |                       | 0 | 1 | 2 |
#           | 5 | 0 | 6 |                       | 3 | 4 | 5 |
#           | 8 | 3 | 1 |                       | 6 | 7 | 8 |
#           
# 상태는 각 위치 인덱스(0부터 8까지)에 위치한 숫자 값들의 튜플로 표현함. 위 예의 경우 초기 상태는 `(7, 2, 4, 5, 0, 6, 8, 3, 1)`로 표현됨.
# 
# 행동은 빈 공간으로 움직일 숫자 타일의 위치 인덱스로 표현함. 위 예의 초기 상태에서 숫자 타일 3(위치 인덱스 7)을 위로 움직여서 다음과 같은 상태로 이행시키는 행동은 `7`로 표현됨.
# 
#           | 7 | 2 | 4 | 
#           | 5 | 3 | 6 |
#           | 8 | 0 | 1 |

# In[ ]:


from itertools import combinations

class EightPuzzle(Problem):
    """8-퍼즐 문제. 3*3 보드.
     상태: 길이가 9인 튜플. 인덱스 i의 원소 값은 i 위치에 놓인 숫자를 의미함. 0은 빈 공간을 의미함."""

    def __init__(self, initial, goal=(0, 1, 2, 3, 4, 5, 6, 7, 8)):
        """초기 상태, 목표 상태 설정"""
        assert inversions(initial) % 2 == inversions(goal) % 2 # 해결 가능한 숫자 배열인지 확인
        self.initial, self.goal = initial, goal
    
    def actions(self, state):
        """빈 공간으로 움직일 숫자 타일의 위치 인덱스."""
        # 빈 공간의 위치에 따라 가능한 행동 정의
        moves = ((1, 3),    (0, 2, 4),    (1, 5),
                 (0, 4, 6), (1, 3, 5, 7), (2, 4, 8),
                 (3, 7),    (4, 6, 8),    (7, 5))
        blank = state.index(0) # 빈 공간의 위치 인덱스: 값 0이 몇번째에 위치해 있는가?
        return moves[blank]
    
    def result(self, state, action):
        """빈 공간과 action에 지정된 위치 인덱스와 위치 교환."""
        s = list(state)
        blank = state.index(0)
        s[action], s[blank] = s[blank], s[action]
        return tuple(s)

    def h(self, node):
        """기본 휴리스틱 함수 지정"""
        return self.manhattan_h(node)

    def mismatch_h(self, node):
        """불일치 타일 수 휴리스틱"""
        return sum(a != b for a, b in zip(node.state, self.goal))

    def manhattan_h(self, node):
        """맨해튼 거리 휴리스틱.
        목표 상태와 비교했을 때 각 숫자 타일별 (가로 위치 차이) + (세로 위치 차이)의 총합"""
        X = (0, 1, 2, 0, 1, 2, 0, 1, 2) # 가로 위치
        Y = (0, 0, 0, 1, 1, 1, 2, 2, 2) # 세로 위치
        return sum(abs(X[s] - X[g]) + abs(Y[s] - Y[g]) 
                   for (s, g) in zip(node.state, self.goal) if s != 0)

    def max_h(self, node):
        score1 = self.manhattan_h(node)
        score2 = self.mismatch_h(node)
        return max(score1, score2)

    
def inversions(board):
    """각 숫자 타일 쌍에 대해 앞쪽에 위치한 숫자 타일이 뒷쪽에 위치한 숫자 타일보다 더 큰 쌍의 개수"""
    return sum((a > b and a != 0 and b != 0) for (a, b) in combinations(board, 2))


def board8(board, fmt=(3 * '{} {} {}\n')):
    "8-퍼즐 보드 출력용 함수"
    return fmt.format(*board).replace('0', '_')


# ## A* 탐색을 이용한 8 퍼즐 문제 해결

# In[ ]:


e1 = EightPuzzle((1, 4, 2, 0, 7, 5, 3, 6, 8))
e2 = EightPuzzle((1, 2, 3, 4, 5, 6, 7, 8, 0))
e3 = EightPuzzle((4, 0, 2, 5, 1, 3, 7, 8, 6))
e4 = EightPuzzle((7, 2, 4, 5, 0, 6, 8, 3, 1))
e5 = EightPuzzle((8, 6, 7, 2, 5, 4, 3, 0, 1))


# In[ ]:


get_ipython().run_cell_magic('time', '', "# 문제 정의에 포함된 기본 휴리스틱 함수(맨해튼 거리) 이용\nsoln = path_states(astar_search(e4))\nprint(f'length: {len(soln)}')\nfor s in soln:\n    print(board8(s))")


# In[ ]:


get_ipython().run_cell_magic('time', '', "# 불일치 타일 수 휴리스틱 이용\nsoln = path_states(astar_search(e4, e4.mismatch_h))\nprint(f'length: {len(soln)}')\nfor s in soln:\n    print(board8(s))")


# In[ ]:




