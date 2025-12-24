import json
import math


class DataLoader:
    def __init__(self, network_path, demand_path):
        self.network_path = network_path
        self.demand_path = demand_path
        self.nodes = {}  # 节点字典：{节点名: (x, y)}
        self.node_index = {}  # 节点名到索引的映射：{节点名: 索引}
        self.links = []  # 路段列表：[(起点索引, 终点索引, 通行能力, 最大限速, 长度, 自由流时间)]
        self.link_id_map = {}  # 路段ID映射：{("A","B"): 索引, ("B","A"): 索引}（双向）

    def load_data(self):
        """加载并解析路网和需求数据"""
        self._load_network()
        self._load_demand()
        return self.nodes, self.links, self.demand, self.link_id_map

    def _load_network(self):
        """解析路网数据"""
        with open(self.network_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        # 解析节点
        node_names = data["nodes"]["names"]
        x_coords = data["nodes"]["x"]
        y_coords = data["nodes"]["y"]
        for idx, name in enumerate(node_names):
            self.nodes[name] = (x_coords[idx], y_coords[idx])
            self.node_index[name] = idx

        # 解析路段
        for link in data["links"]:
            u_name, v_name = link["between"][0], link["between"][1]
            u = self.node_index[u_name]
            v = self.node_index[v_name]
            cap = link["capacity"]
            speedmax = link["speedmax"]

            # 计算路段长度（欧几里得距离，单位：千米）
            length = math.hypot(
                self.nodes[u_name][0] - self.nodes[v_name][0],
                self.nodes[u_name][1] - self.nodes[v_name][1]
            )

            # 计算自由流时间t0 = 长度 / 限速（单位：小时）
            t0 = length / speedmax

            # 双向路段：添加正向和反向
            self.links.append((u, v, cap, speedmax, length, t0))
            self.links.append((v, u, cap, speedmax, length, t0))

            # 记录路段ID映射
            self.link_id_map[(u_name, v_name)] = len(self.links) - 2
            self.link_id_map[(v_name, u_name)] = len(self.links) - 1

    def _load_demand(self):
        """解析出行需求数据"""
        with open(self.demand_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        self.demand = []
        for req in data["demands"]:
            origin = self.node_index[req["from"]]
            dest = self.node_index[req["to"]]
            amount = req["amount"]
            self.demand.append((origin, dest, amount))