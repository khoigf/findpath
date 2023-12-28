# Graph:
# Node: A1 (21.027033, 105.847776), A2 (21.026388, 105.850091), A3 (21.026254, 105.850564), A4 (21.025954, 105.851644), A5 (21.025947, 105.851692)
#       B1 (21.028115, 105.847772), B2 (21.028597, 105.847777), B3 (21.029148, 105.847787), B4 (21.029805, 105.847786), B5 (21.029985, 105.847725),
#       C1 (21.030441, 105.847473), C2 (21.031061, 105.848641), C3 (21.031321, 105.849129), C4 (21.032319, 105.850702), C5 (21.032299, 105.851307)
#       D1 (21.025477, 105.853222), D2 (21.030444, 105.853872), D3 (21.031656, 105.853113), D4 (21.031883, 105.851727),
#       E1 (21.031778, 105.851371), E2 (21.031389, 105.851005), E3 (21.030464, 105.851067), E4 (21.027843, 105.851173),
#       F1 (21.027896, 105.851214), F2 (21.029191, 105.850580), F3 (21.029906, 105.850190), F4 (21.030238, 105.850683),
#       G1 (21.030907, 105.850123), G2 (21.031268, 105.850074), G3 (21.031750, 105.850872),
#       H1 (21.029348, 105.848273), H2 (21.029512, 105.848620), H3 (21.029593, 105.848795), 
#       I1 (21.026941, 105.850294), I2 (21.027795, 105.849982), I3 (21.028865, 105.849574), I4 (21.029172, 105.849434), I5 (21.029761, 105.849184), I6 (21.030566, 105.848855),
#       K1 (21.028605, 105.848088), K2 (21.028990, 105.848452), K3 (21.029227, 105.848975),
#       L1 (21.028731, 105.848578), L2 (21.029006, 105.849085),
#       M1 (21.027577, 105.848959), M2 (21.028426, 105.848720),  M3 (21.028660, 105.848565),  
#       N1 (21.029874, 105.848064), N2 (21.029945, 105.848384),        
# Matrix_adj:
#       A1 -> B1
#       A2 -> A1, I1
#       A3 -> A2
#       A4 -> A3
#       A5 -> A4, D1
#       B1 -> B2, L1
#       B2 -> B3, K1
#       B3 -> B4, H1
#       B4 -> B5, N1
#       B5 -> C1
#       C2 -> C1
#       C3 -> C2, F3
#       C4 -> C5, C3, G3
#       C5 -> C4, E1
#       D1 -> A5, D2
#       D2 -> D3
#       D3 -> D4
#       D4 -> C5, E1
#       E1 -> E2
#       E2 -> E3
#       E3 -> E4, F4
#       E4 -> A4
#       F1 -> A5
#       F2 -> F1, I3
#       F3 -> F2
#       F4 -> F3, G1, E3
#       G1 -> G2, F4
#       G2 -> G3
#       G3 -> E2
#       H1 -> B3, H2, K2
#       H2 -> H1, H3, N2
#       H3 -> H2, I5, K3
#       K1 -> B2, K2
#       K2 -> K1, K3, H1, L1
#       K3 -> K2, H3, L2 
#       L1 -> L2, K2
#       L2 -> K3, I4
#       M1 -> I2, M2
#       M2 -> M1, M3, I3
#       M3 -> I4, M2
#       I1 -> A3, I2
#       I2 -> I3, M1
#       I3 -> I4, M2, F2
#       I4 -> I5, L2, M3
#       I5 -> I6, H3
#       I6 -> C2, B5
#       N1 -> B4
#       N2 -> H2


from queue import PriorityQueue
from tkinter import messagebox
from tkintermapview import TkinterMapView
import networkx as nx
import heapq
import math
import customtkinter
import time
import numpy as np

customtkinter.set_default_color_theme("green")

class App(customtkinter.CTk):
    APP_NAME = "Find Path in Map"
    WIDTH = 800
    HEIGHT = 500

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        global algorithm
        algorithm = "A*"

        self.debug = False
        self.start_point = None
        self.target_point = None

        self.title(App.APP_NAME)
        self.geometry(str(App.WIDTH) + "x" + str(App.HEIGHT))
        self.minsize(App.WIDTH, App.HEIGHT)
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.createcommand("tk::mac::Quit", self.on_closing)

        self.marker_list = []
        self.graph = nx.DiGraph()

        # ============ create two CTkFrames ============

        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        self.frame_left = customtkinter.CTkFrame(
            master=self, width=150, corner_radius=0, fg_color=None
        )
        self.frame_left.grid(row=0, column=0, padx=0, pady=0, sticky="nsew")

        self.frame_right = customtkinter.CTkFrame(master=self, corner_radius=0)
        self.frame_right.grid(row=0, column=1, rowspan=1, pady=0, padx=0, sticky="nsew")

        # ============ frame_left ============

        self.frame_left.grid_rowconfigure(2, weight=1)

        self.button_2 = customtkinter.CTkButton(
            master=self.frame_left,
            text="Clear Markers",
            command=self.clear_marker_event,
        )
        self.button_2.grid(pady=(20, 0), padx=(20, 20), row=0, column=0)

        # Trước mắt chỉ dùng thuật toán A* nên chưa hiển thị select thay đổi thuật toán
        self.btn_change_test = customtkinter.CTkOptionMenu(
            self.frame_left,
            values=[
                "Find Shortest Path By A*",
                "Find Shortest Path By Dijkstra",
            ],
            command=self.change_test,
        )
        self.btn_change_test.grid(row=1, column=0, padx=(20, 20), pady=(10, 0))

        self.test = customtkinter.CTkButton(
            self.frame_left,
            text="Find sortest path",
            command=self.find_shortest_path,
        )

        self.test.grid(row=2, column=0, padx=(20, 20), pady=(10, 0))

        self.map_label = customtkinter.CTkLabel(
            self.frame_left, text="Tile Server:", anchor="w"
        )
        self.map_label.grid(row=3, column=0, padx=(20, 20), pady=(20, 0))

        # self.map_option_menu = customtkinter.CTkOptionMenu(
        #     self.frame_left,
        #     values=["OpenStreetMap"],
        #     command=self.change_map,
        # )
        # self.map_option_menu.grid(row=4, column=0, padx=(20, 20), pady=(10, 0))

        self.map_option_menu = customtkinter.CTkButton(
            master=self.frame_left, text="OpenStreetMap", width=90, command=self.change_map
        )
        self.map_option_menu.grid(row=4, column=0, padx=(20, 20), pady=(0, 0))


        self.appearance_mode_label = customtkinter.CTkLabel(
            self.frame_left, text="Appearance Mode:", anchor="w"
        )
        self.appearance_mode_label.grid(row=5, column=0, padx=(20, 20), pady=(20, 0))
        self.appearance_mode_optionemenu = customtkinter.CTkOptionMenu(
            self.frame_left,
            values=["Light", "Dark", "System"],
            command=self.change_appearance_mode,
        )
        self.appearance_mode_optionemenu.grid(
            row=6, column=0, padx=(20, 20), pady=(10, 20)
        )
        
        # self.map_option_menu.set("OpenStreetMap")

        self.appearance_mode_optionemenu.set("Dark")

        # ============ frame_right ============

        self.frame_right.grid_rowconfigure(1, weight=1)
        self.frame_right.grid_rowconfigure(0, weight=0)
        self.frame_right.grid_columnconfigure(0, weight=1)
        self.frame_right.grid_columnconfigure(1, weight=0)
        self.frame_right.grid_columnconfigure(2, weight=1)

        self.map_widget = TkinterMapView(self.frame_right, corner_radius=0)
        self.map_widget.grid(
            row=1,
            rowspan=1,
            column=0,
            columnspan=3,
            sticky="nswe",
            padx=(0, 0),
            pady=(0, 0),
        )
        self.map_widget.add_right_click_menu_command(
            label="Add Marker", command=self.add_marker_event, pass_coords=True
        )

        self.map_widget.set_polygon(
            [
              (21.027042, 105.847688), 
              (21.026341, 105.850080), (21.025854, 105.851685), 
              (21.025384, 105.853237), (21.026043, 105.853390),  
              (21.027309, 105.853602), (21.027859, 105.853680),
              (21.028821, 105.853812), (21.030489, 105.853912),
              (21.031791, 105.853122), (21.032222, 105.852019), 
              (21.032312, 105.850696), (21.031334, 105.849123), 
              (21.031072, 105.848632), (21.030423, 105.847433), 
              (21.029803, 105.847785), (21.028599, 105.847761), 
              (21.027688, 105.847788), 
            ],
            fill_color=None,
            outline_color="red",
            border_width=5,
            name="phuong_hang_trong_polygon",
        )

        self.entry = customtkinter.CTkEntry(
            master=self.frame_right, placeholder_text="type address"
        )
        self.entry.grid(row=0, column=0, sticky="we", padx=(12, 0), pady=12)
        self.entry.bind("<Return>", self.search_event)

        self.button_8 = customtkinter.CTkButton(
            master=self.frame_right, text="Search", width=90, command=self.search_event
        )
        self.button_8.grid(row=0, column=1, sticky="w", padx=(12, 0), pady=12)

        # Set default values
        self.map_widget.set_address(
            "Hang Trong, Hoan Kiem, Ha Noi, Vietnam", text="Phuong Hang Trong, Ha Noi"
        )

        # ===================BUILD_GRAPH===========================

        self.nodes = [
        #49-node   
        (21.027033, 105.847776), (21.026388, 105.850091),
        (21.026254, 105.850564), (21.025954, 105.851644), 
        (21.025947, 105.851692), (21.028115, 105.847772),
        (21.028597, 105.847777), (21.029148, 105.847787), 
        (21.029805, 105.847786), (21.029985, 105.847725),
        (21.030441, 105.847473), (21.031061, 105.848641),
        (21.031321, 105.849129), (21.032319, 105.850702),
        (21.032299, 105.851307), (21.025477, 105.853222),
        (21.030444, 105.853872), (21.031656, 105.853113),
        (21.031883, 105.851727), (21.031778, 105.851371), 
        (21.031389, 105.851005), (21.030464, 105.851067),
        (21.027843, 105.851173), (21.027896, 105.851214),
        (21.029191, 105.850580), (21.029906, 105.850190),
        (21.030238, 105.850683), (21.030907, 105.850123), 
        (21.031268, 105.850074), (21.031750, 105.850872),
        (21.029348, 105.848273), (21.029512, 105.848620),
        (21.029593, 105.848795), (21.026941, 105.850294), 
        (21.027795, 105.849982), (21.028865, 105.849574),
        (21.029172, 105.849434), (21.029761, 105.849184),
        (21.030566, 105.848855), (21.028605, 105.848088),
        (21.028990, 105.848452), (21.029227, 105.848975),
        (21.028731, 105.848578), (21.029006, 105.849085),
        (21.028660, 105.848565), (21.028426, 105.848720), 
        (21.027577, 105.848959), (21.029874, 105.848064),
        (21.029945, 105.848384)
        ]

        self.G = { 
        # A1 -> B1
            (21.027033, 105.847776): [
                (21.028115, 105.847772),
            ],

        # A2 -> A1, I1
            (21.026388, 105.850091): [
                (21.027033, 105.847776), (21.026941, 105.850294),
            ],
        # A3 -> A2
            (21.026254, 105.850564): [
                (21.026388, 105.850091),
            ],
        # A4 -> A3
            (21.025954, 105.851644): [
                (21.026254, 105.850564),
            ],
        # A5 -> A4, D1
             (21.025947, 105.851692): [
                (21.025954, 105.851644), (21.025477, 105.853222),
            ],
        # B1 -> B2, L1
            (21.028115, 105.847772): [
                (21.028597, 105.847777), (21.028731, 105.848578),
            ],
        # B2 -> B3, K1
            (21.028597, 105.847777): [
                (21.029148, 105.847787), (21.028605, 105.848088),
            ], 
        # B3 -> B4, H1
            (21.029148, 105.847787): [
                (21.029805, 105.847786), (21.029348, 105.848273),
            ],
        # B4 -> B5, N1
            (21.029805, 105.847786): [
                (21.029985, 105.847725), (21.029874, 105.848064),
            ],
        # B5 -> C1
            (21.029985, 105.847725): [
                (21.030441, 105.847473),
            ],
        # C2 -> C1
            (21.031061, 105.848641): [
                (21.030441, 105.847473),
            ],
        # C3 -> C2, F3
            (21.031321, 105.849129): [
                (21.031061, 105.848641), (21.029906, 105.850190),
            ],
        # C4 -> C5, C3, G3
            (21.032319, 105.850702): [
                (21.032299, 105.851307), (21.031321, 105.849129), (21.031750, 105.850872),
            ],
        # C5 -> C4, E1
            (21.032299, 105.851307): [
                (21.032319, 105.850702), (21.031778, 105.851371),
            ],
        # D1 -> A5, D2
            (21.025477, 105.853222): [
                (21.025947, 105.851692), (21.030444, 105.853872),
            ],
        # D2 -> D3
            (21.030444, 105.853872): [
                (21.031656, 105.853113),
            ],
        # D3 -> D4
            (21.031656, 105.853113): [
                (21.031883, 105.851727),
            ],
        # D4 -> C5, E1
            (21.031883, 105.851727): [
                (21.032299, 105.851307), (21.031778, 105.851371),
            ],
        # E1 -> E2
            (21.031778, 105.851371): [
                (21.031389, 105.851005),
            ],
        # E2 -> E3
            (21.031389, 105.851005): [
                (21.030464, 105.851067),
            ],
        # E3 -> E4, F4
            (21.030464, 105.851067): [
                (21.027843, 105.851173), (21.030238, 105.850683),
            ],
        # E4 -> A4
            (21.027843, 105.851173): [
                (21.025954, 105.851644),
            ],
        # F1 -> A5
            (21.027896, 105.851214): [
                (21.025947, 105.851692),
            ],
        # F2 -> F1, I3
            (21.029191, 105.850580): [
                (21.027896, 105.851214), (21.028865, 105.849574),
            ],
        # F3 -> F2
            (21.029906, 105.850190): [
                (21.029191, 105.850580),
            ],
        # F4 -> F3, G1, E3
            (21.030238, 105.850683): [
                (21.029906, 105.850190), (21.030907, 105.850123), (21.030464, 105.851067),
            ],
        # G1 -> G2, F4
            (21.030907, 105.850123): [
                (21.031268, 105.850074), (21.030238, 105.850683),
            ],
        # G2 -> G3
            (21.031268, 105.850074): [
                (21.031750, 105.850872),
            ],
        # G3 -> E2
            (21.031750, 105.850872): [
                (21.031389, 105.851005),
            ],
        # H1 -> B3, H2, K2
            (21.029348, 105.848273): [
                (21.029148, 105.847787), (21.029512, 105.848620), (21.028990, 105.848452),
            ],
        # H2 -> H1, H3, N2
            (21.029512, 105.848620): [
                (21.029348, 105.848273), (21.029593, 105.848795), (21.029945, 105.848384),
            ],
        # H3 -> H2, I5, K3
            (21.029593, 105.848795): [
                (21.029512, 105.848620), (21.029761, 105.849184), (21.029227, 105.848975),
            ],
        # K1 -> B2, K2
            (21.028605, 105.848088): [
                (21.028597, 105.847777), (21.028990, 105.848452),
            ],
        # K2 -> K1, K3, H1, L1
            (21.028990, 105.848452): [
                (21.028605, 105.848088), (21.029227, 105.848975), (21.029348, 105.848273), (21.028731, 105.848578),
            ],
        # K3 -> K2, H3, L2 
            (21.029227, 105.848975): [
                (21.028990, 105.848452), (21.029593, 105.848795), (21.029006, 105.849085),
            ],
        # L1 -> L2, K2
            (21.028731, 105.848578): [
                (21.029006, 105.849085), (21.028990, 105.848452),
            ],
        # L2 -> K3, I4
            (21.029006, 105.849085): [
                (21.029227, 105.848975), (21.029172, 105.849434),
            ],
        # M1 -> I2, M2
            (21.027577, 105.848959): [
                (21.027795, 105.849982), (21.028426, 105.848720),
            ],
        # M2 -> M1, M3, I3
            (21.028426, 105.848720): [
                (21.027577, 105.848959), (21.028660, 105.848565), (21.028865, 105.849574),
            ],
        # M3 -> I4, M2
            (21.028660, 105.848565): [
                (21.029172, 105.849434), (21.028426, 105.848720),
            ],
        # I1 -> A3, I2
            (21.026941, 105.850294): [
                (21.026254, 105.850564), (21.027795, 105.849982),
            ],
        # I2 -> I3, M1
            (21.027795, 105.849982): [
                (21.028865, 105.849574), (21.027577, 105.848959),
            ],
        # I3 -> I4, M2, F2
            (21.028865, 105.849574): [
                (21.029172, 105.849434), (21.028426, 105.848720), (21.029191, 105.850580),
            ],
        # I4 -> I5, L2, M3
            (21.029172, 105.849434): [
                (21.029761, 105.849184), (21.029006, 105.849085), (21.028660, 105.848565),
            ],
        # I5 -> I6, H3
            (21.029761, 105.849184): [
                (21.030566, 105.848855), (21.029593, 105.848795),
            ],
        # I6 -> C2, B5
            (21.030566, 105.848855): [
                (21.031061, 105.848641), (21.029985, 105.847725),
            ],
        # N1 -> B4
            (21.029874, 105.848064): [
                (21.029805, 105.847786),
            ],
        # N2 -> H2
            (21.029945, 105.848384): [
                (21.029512, 105.848620),
            ],
        
        }
        self.make_graph()

    def make_graph(self):
        self.graph.clear()
        self.graph.add_nodes_from(self.nodes)
        for node in list(self.G.keys()):
            for neighbor in self.G[node]:
                self.graph.add_edge(
                    node, neighbor, weight=self.calculate_distance(node, neighbor)
                )

    # ============================Các hàm tính toán===============================

    def calculate_distance(self, node, neighbor):
        return np.linalg.norm(np.array(node) - np.array(neighbor))

    # =========================Các thuật toán tìm kiếm==============================
    
    def find_shortest_path(self):
        start_time = time.time()
        if self.start_point == None or self.target_point == None:
            messagebox.showerror("Error", "missing entry point !")
            return
        global algorithm
        path = None
        if algorithm == "A*":
            path = self.astar(self.graph, self.start_point, self.target_point)
        elif algorithm == "Dijkstra":
            path = self.dijkstra(self.graph, self.start_point, self.target_point)
        else:
            messagebox.showerror("Error", "Can't find algorithm: "+algorithm)
            return

        if path != None:
            cur_point = None
            for node in path:
                if cur_point != None:
                    self.draw_dash_line((cur_point, node))
                cur_point = node
            run_time = (time.time() - start_time) * 1000
            messagebox.showinfo("Kết quả", f"Tìm kiếm bằng {algorithm} trong {run_time} ms")
        else:
            messagebox.showwarning("Not found", "Không thể tìm đường đi trong phạm vi này !")

    def dijkstra(self, graph, start, goal):
        distance = { node: float('infinity') for node in graph.nodes() }
        distance[start] = 0
        came_from = {}
        open_set = PriorityQueue()
        open_set.put((0, start))

        while not open_set.empty():
            cur_distance, cur_node = open_set.get()

            if (cur_distance > distance[cur_node]):
                continue

            for neighbor in graph.neighbors(cur_node):
                distance_to_neighbor = cur_distance + graph.get_edge_data(cur_node, neighbor)['weight']
                if distance_to_neighbor < distance[neighbor]:
                    distance[neighbor] = distance_to_neighbor
                    open_set.put((distance_to_neighbor, neighbor))
                    came_from[neighbor] = cur_node
        
        if goal in came_from:
            return self.reconstruct_path(came_from, start, goal)
        else: 
            return None

    def astar(self, graph, start, goal):
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        id = 0

        while not open_set.empty():
            id += 1
            current = open_set.get()[1]

            if current == goal:
                path = self.reconstruct_path(came_from, start, goal)
                return path

            for neighbor in graph.neighbors(current):
                new_cost = cost_so_far[current] + graph.get_edge_data(current, neighbor)['weight']
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(goal, neighbor)
                    open_set.put((priority, neighbor))
                    came_from[neighbor] = current

        return None

    def heuristic(self, a, b):
        # khoảng cách theo đường chim bay
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []

        while current != start:
            path.append(current)
            current = came_from[current]

        path.append(start)
        path.reverse()
        return path

    # =================Các hàm xử lý sự kiện button==================

    def handle_a_star(self):
        global algorithm
        algorithm = "A*"
        # self.find_shortest_path()

    def handle_dijkstra(self):
        global algorithm
        algorithm = "Dijkstra"
        # self.find_shortest_path()

    # =====================Các hàm xử lý giao diện==========================

    def search_event(self, event=None):
        self.map_widget.set_address(self.entry.get())

    def clear_marker_event(self):
        self.start_point = None
        self.target_point = None
        self.marker_list.clear()
        self.map_widget.delete_all_marker()
        self.map_widget.delete_all_path()
        self.make_graph()
        if self.debug:
            for node in self.graph.nodes():
                new_marker = self.map_widget.set_marker(node[0], node[1], text="")
                self.marker_list.append(new_marker)
            for edge in self.graph.edges():
                self.draw_dash_line(edge)

    def change_appearance_mode(self, new_appearance_mode: str):
        customtkinter.set_appearance_mode(new_appearance_mode)

    def change_map(self, new_map: str):
        self.map_widget.set_tile_server(
            "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png"
        )

    # Hàm thay đổi thuật thoán
    def change_test(self, new_test: str):
        self.map_widget.delete_all_path()
        if new_test == "Find Shortest Path By A*":
            self.handle_a_star()
        elif new_test == "Find Shortest Path By Dijkstra":
            self.handle_dijkstra()

    def add_marker_event(self, coords):
        if len(self.marker_list) == 2 or len(self.marker_list) == 0:
            self.clear_marker_event()
            print("Start:", coords)
            start_node = self.find_nearest_point_on_edges(coords)
            self.start_point = start_node
            new_marker = self.map_widget.set_marker(start_node[0], start_node[1], text="Start")
        else:
            print("Goal:", coords)
            goal_node = self.find_nearest_point_on_edges(coords)
            self.target_point = goal_node
            new_marker = self.map_widget.set_marker(goal_node[0], goal_node[1], text="End")

        self.marker_list.append(new_marker)

    def find_nearest_point_on_edges(self, coords):
        min_distance = float('inf')
        at_edge = None
        nearest_point = None

        for edge in self.graph.edges():
            start, end = edge
            start_coords = np.array(start)
            end_coords = np.array(end)
            point_coords = np.array(coords)

            edge_length = np.linalg.norm(end_coords - start_coords)
            
            # vecto hinh chieu tu coords xuong edge 
            projection_vector = (np.dot(point_coords - start_coords, end_coords - start_coords) / edge_length**2) * (end_coords - start_coords)
            projection = projection_vector + start_coords

            # kiểm tra xem hình chiếu có thuộc edge hay ko, bằng cách so sánh góc giữa 2 vector
            if 0 <= np.dot(projection - start, end_coords - start_coords) <= edge_length**2:
                distance = np.linalg.norm(np.array(projection) - point_coords)
                if (distance < min_distance) :
                    min_distance = distance
                    nearest_point = projection
                    at_edge = edge
            else:
                continue

        # add điểm đc chọn vào graph, cần define lại cạnh chứa điểm đó
        node = tuple(nearest_point)
        self.graph.remove_edge(at_edge[0], at_edge[1])
        self.graph.add_node(node)
        self.graph.add_edge(node, at_edge[0], weight=self.calculate_distance(node, at_edge[0]))
        self.graph.add_edge(node, at_edge[1], weight=self.calculate_distance(node, at_edge[1]))
        self.graph.add_edge(at_edge[0], node, weight=self.calculate_distance(node, at_edge[0]))
        self.graph.add_edge(at_edge[1], node, weight=self.calculate_distance(node, at_edge[1]))
        return node

    
    def display_map(self, path):
        for i in range(len(path) - 1):
            print(path[i], " -> ")
            self.map_widget.set_path([path[i], path[i + 1]])
        print(path[len(path) - 1])

    def draw_dash_line(self, line):
        start, end = line[0], line[1]
        x1, y1 = start[0], start[1]
        x2, y2 = end[0], end[1]
        d1 = (x2 - x1)/10
        d2 = (y2 - y1)/10
        
        for i in range(10):
            if i % 2 == 0:
                self.map_widget.set_path([(x1, y1), (x1 + d1, y1 + d2)])
    
            x1 = x1 + d1
            y1 = y1 + d2

    def on_closing(self, event=0):
        self.destroy()

    def start(self):
        self.mainloop()


if __name__ == "__main__":
    app = App()
    app.start()