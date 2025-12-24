import heapq
from typing import List, Tuple, Optional, Dict
import math

# 类型别名定义（提升代码可读性）
NodeIdx = int  # 节点索引类型
LinkIdx = int  # 路段索引类型
FlowValue = float  # 流量值类型
TimeValue = float  # 时间值类型
ODDemand = Tuple[NodeIdx, NodeIdx, FlowValue]  # (起点索引, 终点索引, 交通量)
LinkData = Tuple[NodeIdx, NodeIdx, FlowValue, float, float, TimeValue]  # (u, v, 通行能力, 最大限速, 长度, 自由流时间)
PathResult = Tuple[List[NodeIdx], List[LinkIdx]]  # (节点路径, 路段索引路径)


class ShortestPathSolver:
    """最短路径求解器（独立模块，负责所有路径计算逻辑）"""
    def __init__(self, links: List[LinkData], num_nodes: int):
        self.links = links  # 路段数据列表
        self.num_nodes = num_nodes  # 节点总数
        self._precompute_link_adj()  # 预计算邻接表

    def _precompute_link_adj(self):
        """预计算邻接表：优化路径搜索时的路段遍历效率"""
        self.adj: Dict[NodeIdx, List[Tuple[NodeIdx, LinkIdx]]] = {idx: [] for idx in range(self.num_nodes)}
        for link_idx, (u, v, _, _, _, _) in enumerate(self.links):
            self.adj[u].append((v, link_idx))  # 记录每个节点的出边（节点+路段索引）

    def dijkstra(self, origin: NodeIdx, dest: NodeIdx, link_flows: Optional[List[FlowValue]] = None) -> PathResult:
        """
        Dijkstra算法计算最短路径（支持考虑拥堵和不考虑拥堵两种模式）
        :param origin: 起点节点索引
        :param dest: 终点节点索引
        :param link_flows: 路段流量（None时使用自由流时间，否则计算拥堵时间）
        :return: (节点路径列表, 路段索引列表)
        """
        # 初始化距离数组（默认无穷大）和前驱记录
        dist: List[TimeValue] = [math.inf] * self.num_nodes
        prev_node: List[NodeIdx] = [-1] * self.num_nodes  # 前驱节点索引
        prev_link: List[LinkIdx] = [-1] * self.num_nodes  # 前驱路段索引
        dist[origin] = 0.0

        # 优先队列：(当前累计时间, 当前节点索引)，按时间升序排列
        priority_queue: List[Tuple[TimeValue, NodeIdx]] = []
        heapq.heappush(priority_queue, (0.0, origin))

        while priority_queue:
            current_time, u = heapq.heappop(priority_queue)

            # 终止条件：到达终点或当前路径已非最短
            if u == dest:
                break
            if current_time > dist[u]:
                continue

            # 遍历当前节点的所有出边
            for v, link_idx in self.adj[u]:
                # 计算当前路段的行程时间
                section_time = self._calculate_section_time(link_idx, link_flows)

                # 松弛操作：更新最短路径
                if dist[v] > dist[u] + section_time:
                    dist[v] = dist[u] + section_time
                    prev_node[v] = u
                    prev_link[v] = link_idx
                    heapq.heappush(priority_queue, (dist[v], v))

        # 回溯重建路径
        node_path, link_path = self._reconstruct_path(dest, prev_node, prev_link)
        return node_path, link_path

    def _calculate_section_time(self, link_idx: LinkIdx, link_flows: Optional[List[FlowValue]]) -> TimeValue:
        """
        计算路段行程时间（支持自由流和拥堵两种场景）
        :param link_idx: 路段索引
        :param link_flows: 路段流量（None时返回自由流时间）
        :return: 路段行程时间（小时）
        """
        _, _, cap, _, _, t0 = self.links[link_idx]  # 提取路段关键参数
        if link_flows is None:
            return t0  # 不考虑拥堵：直接返回自由流时间
        else:
            # 考虑拥堵：使用公式 t(q) = t0 * (1 + q/cap)^2
            q = link_flows[link_idx]
            return t0 * ((1 + q / cap) ** 2)

    @staticmethod
    def _reconstruct_path(dest: NodeIdx, prev_node: List[NodeIdx], prev_link: List[LinkIdx]) -> PathResult:
        """
        从终点回溯重建路径
        :param dest: 终点节点索引
        :param prev_node: 前驱节点数组
        :param prev_link: 前驱路段数组
        :return: (节点路径列表, 路段索引列表)
        """
        node_path: List[NodeIdx] = []
        link_path: List[LinkIdx] = []
        current = dest

        # 回溯至起点（prev_node为-1时停止）
        while current != -1:
            node_path.append(current)
            if prev_link[current] != -1:
                link_path.append(prev_link[current])
            current = prev_node[current]

        # 反转路径（从起点→终点）
        node_path.reverse()
        link_path.reverse()
        return node_path, link_path


class TrafficAssignmentCore:
    """交通分配核心类（整合三种分配算法，职责单一）"""
    def __init__(self, links: List[LinkData], demands: List[ODDemand], num_nodes: int):
        self.links = links  # 路段数据
        self.demands = demands  # 出行需求
        self.num_nodes = num_nodes  # 节点总数
        self.num_links = len(links)  # 路段总数（含双向）

        # 核心状态：路段流量（所有算法共享此状态）
        self.link_flows: List[FlowValue] = [0.0 for _ in range(self.num_links)]

        # 初始化最短路径求解器（复用模块）
        self.path_solver = ShortestPathSolver(links, num_nodes)

    def reset_flows(self) -> None:
        """重置路段流量为初始状态（所有算法执行前调用）"""
        self.link_flows = [0.0 for _ in range(self.num_links)]

    def all_or_nothing_assignment(self) -> List[FlowValue]:
        """
        全有全无分配算法
        核心逻辑：不考虑拥堵，将每个OD对的所有交通量全部分配到自由流最短路径
        :return: 分配后的路段流量列表
        """
        self.reset_flows()

        for origin, dest, amount in self.demands:
            # 计算自由流最短路径（不考虑拥堵）
            _, link_path = self.path_solver.dijkstra(origin, dest, link_flows=None)
            # 分配全部交通量到路径路段
            for link_idx in link_path:
                self.link_flows[link_idx] += amount

        return self.link_flows.copy()

    def incremental_assignment(self, num_steps: int = 10) -> List[FlowValue]:
        """
        增量分配算法
        核心逻辑：将交通量分成num_steps步逐步分配，每步基于当前拥堵状态更新最短路径
        :param num_steps: 增量分配步数（默认10步，步数越多结果越接近均衡）
        :return: 分配后的路段流量列表
        """
        if num_steps <= 0:
            raise ValueError("增量步数必须为正整数")

        self.reset_flows()
        step_ratio = 1.0 / num_steps  # 每步分配的流量比例

        for origin, dest, total_amount in self.demands:
            step_amount = total_amount * step_ratio  # 每步分配的具体流量
            for _ in range(num_steps):
                # 基于当前流量计算拥堵状态下的最短路径
                _, link_path = self.path_solver.dijkstra(origin, dest, link_flows=self.link_flows)
                # 分配当前步流量
                for link_idx in link_path:
                    self.link_flows[link_idx] += step_amount

        return self.link_flows.copy()

    def user_equilibrium_assignment(self, max_iter: int = 100, tol: float = 1e-4) -> List[FlowValue]:
        """
        基于Frank-Wolfe算法的用户均衡（UE）分配
        均衡条件：所有被使用的路径行程时间相等且最小，无出行者可通过换路缩短时间
        :param max_iter: 最大迭代次数（防止无限循环）
        :param tol: 收敛阈值（总行程时间相对变化率）
        :return: 均衡后的路段流量列表
        """
        self.reset_flows()
        prev_total_time = self.calculate_total_travel_time()  # 初始总行程时间

        for iter_idx in range(max_iter):
            # Step 1: 基于当前流量计算辅助流量（全有全无分配）
            aux_flows = self._compute_auxiliary_flows()

            # Step 2: 计算搜索方向（辅助流量 - 当前流量）
            search_direction = [aux - curr for aux, curr in zip(aux_flows, self.link_flows)]

            # Step 3: 一维搜索最优步长λ（0≤λ≤1）
            optimal_lambda = self._line_search_optimal_lambda(search_direction)

            # Step 4: 更新当前流量
            self.link_flows = [
                curr + optimal_lambda * dir_val
                for curr, dir_val in zip(self.link_flows, search_direction)
            ]

            # Step 5: 收敛判断
            current_total_time = self.calculate_total_travel_time()
            time_change_rate = abs(current_total_time - prev_total_time) / prev_total_time
            if time_change_rate < tol:
                print(f"UE算法收敛：迭代次数={iter_idx+1}, 总行程时间={current_total_time:.2f}小时")
                break

            prev_total_time = current_total_time
        else:
            print(f"UE算法未达到收敛阈值（最大迭代次数={max_iter}）")

        return self.link_flows.copy()

    def _compute_auxiliary_flows(self) -> List[FlowValue]:
        """计算辅助流量：基于当前流量的全有全无分配"""
        aux_flows = [0.0 for _ in range(self.num_links)]
        for origin, dest, amount in self.demands:
            _, link_path = self.path_solver.dijkstra(origin, dest, link_flows=self.link_flows)
            for link_idx in link_path:
                aux_flows[link_idx] += amount
        return aux_flows

    def _line_search_optimal_lambda(self, direction: List[FlowValue]) -> float:
        """
        一维搜索最优步长λ（最小化总行程时间）
        核心逻辑：通过二分法求解总行程时间函数的导数为0的点
        :param direction: 搜索方向（辅助流量 - 当前流量）
        :return: 最优步长λ（0≤λ≤1）
        """
        left, right = 0.0, 1.0  # 步长搜索范围
        max_search_iter = 20  # 二分法最大迭代次数

        for _ in range(max_search_iter):
            mid = (left + right) / 2
            deriv = self._total_time_derivative(mid, direction)
            if deriv < 0:
                left = mid  # 导数为负，最优λ在右侧
            else:
                right = mid  # 导数为正，最优λ在左侧

        return (left + right) / 2

    def _total_time_derivative(self, lam: float, direction: List[FlowValue]) -> float:
        """
        计算总行程时间函数在λ处的导数（用于一维搜索）
        导数公式：dZ/dλ = Σ [t(q+λ*d) * d]，其中t为路段行程时间，d为搜索方向
        :param lam: 当前步长参数
        :param direction: 搜索方向
        :return: 导数数值
        """
        derivative = 0.0
        for link_idx in range(self.num_links):
            d = direction[link_idx]
            if d == 0:
                continue  # 方向为0时对导数无贡献，跳过

            # 计算当前步长下的路段流量和行程时间
            q = self.link_flows[link_idx]
            q_lam = q + lam * d  # λ步长下的流量
            _, _, cap, _, _, t0 = self.links[link_idx]
            t_lam = t0 * ((1 + q_lam / cap) ** 2)  # λ步长下的行程时间

            derivative += t_lam * d
        return derivative

    def calculate_total_travel_time(self) -> TimeValue:
        """
        计算路网总出行时间（结果评估核心指标）
        公式：总时间 = Σ (路段流量 * 路段行程时间)
        :return: 总出行时间（小时）
        """
        total_time = 0.0
        for link_idx in range(self.num_links):
            q = self.link_flows[link_idx]
            _, _, cap, _, _, t0 = self.links[link_idx]
            t = t0 * ((1 + q / cap) ** 2)
            total_time += q * t
        return total_time

    def get_single_od_assignment(self, origin_name: str, dest_name: str, node_name_to_idx: Dict[str, NodeIdx]) -> Tuple[List[LinkIdx], List[FlowValue]]:
        """
        单一OD对流量分配（满足报告测试需求）
        :param origin_name: 起点节点名（如"A"）
        :param dest_name: 终点节点名（如"F"）
        :param node_name_to_idx: 节点名到索引的映射字典
        :return: (被使用的路段索引列表, 对应路段流量列表)
        """
        origin = node_name_to_idx[origin_name]
        dest = node_name_to_idx[dest_name]

        # 仅保留目标OD对的需求
        single_demand = [(origin, dest, amount) for o, d, amount in self.demands if o == origin and d == dest]
        if not single_demand:
            raise ValueError(f"未找到{origin_name}→{dest_name}的出行需求")

        # 执行UE分配（最能反映实际流量分布）
        self.reset_flows()
        for o, d, amount in single_demand:
            _, link_path = self.path_solver.dijkstra(o, d, link_flows=self.link_flows)
            for link_idx in link_path:
                self.link_flows[link_idx] += amount

        # 筛选被使用的路段（流量>0）
        used_links = [idx for idx in range(self.num_links) if self.link_flows[idx] > 1e-6]
        used_flows = [self.link_flows[idx] for idx in used_links]
        return used_links, used_flows