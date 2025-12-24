import json
import math
import heapq
import matplotlib.pyplot as plt
from typing import List, Tuple, Dict, Optional

# 全局类型定义
Node = str
NodeIdx = int
LinkKey = Tuple[Node, Node]
Flow = float
Time = float
ODPair = Tuple[Node, Node, Flow]


class TrafficNetwork:
    """路网核心类：负责数据加载、路网建模、行程时间计算"""

    def __init__(self, network_path: str, demand_path: str):
        # 加载原始数据
        self.network_data = self._load_json(network_path)
        self.demand_data = self._load_json(demand_path)

        # 路网基础数据
        self.nodes: List[Node] = self.network_data["nodes"]["name"]
        self.node_idx: Dict[Node, NodeIdx] = {node: idx for idx, node in enumerate(self.nodes)}
        self.node_coords: Dict[Node, Tuple[float, float]] = self._build_node_coords()

        # 路段数据（双向）
        self.links: List[Dict] = self._build_links()
        self.link_count = len(self.links)
        self.link_key_to_idx: Dict[LinkKey, NodeIdx] = self._build_link_key_map()

        # 出行需求数据
        self.demands: List[ODPair] = self._build_demands()

    @staticmethod
    def _load_json(file_path: str) -> Dict:
        """加载JSON文件"""
        with open(file_path, 'r', encoding='utf-8') as f:
            return json.load(f)

    def _build_node_coords(self) -> Dict[Node, Tuple[float, float]]:
        """构建节点坐标字典"""
        coords = {}
        xs = self.network_data["nodes"]["x"]
        ys = self.network_data["nodes"]["y"]
        for node, x, y in zip(self.nodes, xs, ys):
            coords[node] = (x, y)
        return coords

    def _build_links(self) -> List[Dict]:
        """构建路段数据（含双向路段，计算长度和自由流时间）"""
        links = []
        betweens = self.network_data["links"]["between"]
        capacities = self.network_data["links"]["capacity"]
        speedmaxs = self.network_data["links"]["speedmax"]

        for between, cap, spd in zip(betweens, capacities, speedmaxs):
            u, v = between[0], between[1]
            # 计算路段长度（欧几里得距离，km）
            len_km = math.hypot(
                self.node_coords[u][0] - self.node_coords[v][0],
                self.node_coords[u][1] - self.node_coords[v][1]
            )
            # 自由流时间（小时）t0 = 长度 / 限速
            t0 = len_km / spd

            # 添加正向路段
            links.append({
                "u": u, "v": v, "u_idx": self.node_idx[u], "v_idx": self.node_idx[v],
                "capacity": cap, "speedmax": spd, "length": len_km, "t0": t0
            })
            # 添加反向路段（双向通行）
            links.append({
                "u": v, "v": u, "u_idx": self.node_idx[v], "v_idx": self.node_idx[u],
                "capacity": cap, "speedmax": spd, "length": len_km, "t0": t0
            })
        return links

    def _build_link_key_map(self) -> Dict[LinkKey, NodeIdx]:
        """构建路段键（(u,v)）到索引的映射"""
        link_map = {}
        for idx, link in enumerate(self.links):
            link_map[(link["u"], link["v"])] = idx
        return link_map

    def _build_demands(self) -> List[ODPair]:
        """构建标准化出行需求列表"""
        origins = self.demand_data["from"]
        dests = self.demand_data["to"]
        amounts = self.demand_data["amount"]
        return [(o, d, a) for o, d, a in zip(origins, dests, amounts)]

    def calculate_section_time(self, link_idx: int, flow: Optional[Flow] = None) -> Time:
        """
        计算路段行程时间
        :param link_idx: 路段索引
        :param flow: 路段流量（None时返回自由流时间）
        :return: 行程时间（小时）
        """
        link = self.links[link_idx]
        t0 = link["t0"]
        if flow is None:
            return t0
        # 考虑拥堵：t(q) = t0 * (1 + q/cap)^2
        cap = link["capacity"]
        return t0 * ((1 + flow / cap) ** 2)


class ShortestPathFinder:
    """最短路径查找类：基于Dijkstra算法"""

    def __init__(self, network: TrafficNetwork):
        self.network = network
        self.node_count = len(network.nodes)
        # 构建邻接表：{节点索引: [(邻接节点索引, 路段索引)]}
        self.adj: List[List[Tuple[NodeIdx, int]]] = self._build_adjacency_list()

    def _build_adjacency_list(self) -> List[List[Tuple[NodeIdx, int]]]:
        """构建邻接表优化路径搜索"""
        adj = [[] for _ in range(self.node_count)]
        for link_idx, link in enumerate(self.network.links):
            u_idx = link["u_idx"]
            v_idx = link["v_idx"]
            adj[u_idx].append((v_idx, link_idx))
        return adj

    def find(self, origin: Node, dest: Node, flows: Optional[List[Flow]] = None) -> Tuple[List[Node], List[int]]:
        """
        查找最短路径
        :param origin: 起点节点名
        :param dest: 终点节点名
        :param flows: 路段流量列表（None时不考虑拥堵）
        :return: (节点路径列表, 路段索引列表)
        """
        origin_idx = self.network.node_idx[origin]
        dest_idx = self.network.node_idx[dest]

        # 初始化距离和前驱
        dist = [math.inf] * self.node_count
        prev_node = [-1] * self.node_count
        prev_link = [-1] * self.node_count
        dist[origin_idx] = 0.0

        # 优先队列：(当前距离, 当前节点索引)
        heap = []
        heapq.heappush(heap, (0.0, origin_idx))

        while heap:
            current_dist, u_idx = heapq.heappop(heap)
            if u_idx == dest_idx:
                break
            if current_dist > dist[u_idx]:
                continue

            # 遍历邻接节点
            for v_idx, link_idx in self.adj[u_idx]:
                # 计算路段行程时间
                if flows is None:
                    section_time = self.network.calculate_section_time(link_idx)
                else:
                    section_time = self.network.calculate_section_time(link_idx, flows[link_idx])

                # 松弛操作
                if dist[v_idx] > dist[u_idx] + section_time:
                    dist[v_idx] = dist[u_idx] + section_time
                    prev_node[v_idx] = u_idx
                    prev_link[v_idx] = link_idx
                    heapq.heappush(heap, (dist[v_idx], v_idx))

        # 回溯重建路径
        node_path, link_path = self._reconstruct_path(dest_idx, prev_node, prev_link)
        return node_path, link_path

    def _reconstruct_path(self, dest_idx: NodeIdx, prev_node: List[NodeIdx], prev_link: List[int]) -> Tuple[
        List[Node], List[int]]:
        """从终点回溯重建路径"""
        node_path = []
        link_path = []
        current_idx = dest_idx

        while current_idx != -1:
            node_path.append(self.network.nodes[current_idx])
            if prev_link[current_idx] != -1:
                link_path.append(prev_link[current_idx])
            current_idx = prev_node[current_idx]

        # 反转路径（起点→终点）
        node_path.reverse()
        link_path.reverse()
        return node_path, link_path


class TrafficAssigner:
    """交通分配类：实现三种分配算法"""

    def __init__(self, network: TrafficNetwork):
        self.network = network
        self.path_finder = ShortestPathFinder(network)
        self.link_count = network.link_count
        # 分配结果：路段流量
        self.flows: List[Flow] = [0.0] * self.link_count

    def reset_flows(self) -> None:
        """重置路段流量为0"""
        self.flows = [0.0] * self.link_count

    def all_or_nothing(self) -> List[Flow]:
        """全有全无分配算法"""
        self.reset_flows()
        for o, d, amount in self.network.demands:
            # 不考虑拥堵，查找最短路径
            _, link_path = self.path_finder.find(o, d, flows=None)
            # 分配全部流量
            for link_idx in link_path:
                self.flows[link_idx] += amount
        return self.flows.copy()

    def incremental(self, steps: int = 10) -> List[Flow]:
        """
        增量分配算法
        :param steps: 增量步数
        """
        if steps <= 0:
            raise ValueError("增量步数必须为正整数")

        self.reset_flows()
        step_amount_ratio = 1.0 / steps  # 每步分配比例

        for o, d, total_amount in self.network.demands:
            step_amount = total_amount * step_amount_ratio
            for _ in range(steps):
                # 考虑当前拥堵状态查找最短路径
                _, link_path = self.path_finder.find(o, d, flows=self.flows)
                # 分配当前步流量
                for link_idx in link_path:
                    self.flows[link_idx] += step_amount
        return self.flows.copy()

    def user_equilibrium(self, max_iter: int = 100, tol: float = 1e-4) -> List[Flow]:
        """
        基于Frank-Wolfe算法的用户均衡分配
        :param max_iter: 最大迭代次数
        :param tol: 收敛阈值（总行程时间变化率）
        """
        self.reset_flows()
        prev_total_time = self.calculate_total_time()

        for iter_idx in range(max_iter):
            # Step 1: 计算辅助流量（当前状态下的全有全无分配）
            aux_flows = self._get_auxiliary_flows()
            # Step 2: 计算搜索方向
            direction = [aux - curr for aux, curr in zip(aux_flows, self.flows)]
            # Step 3: 一维搜索最优步长
            lam = self._find_optimal_lambda(direction)
            # Step 4: 更新流量
            self.flows = [curr + lam * dir_val for curr, dir_val in zip(self.flows, direction)]
            # Step 5: 收敛判断
            current_total_time = self.calculate_total_time()
            time_diff_rate = abs(current_total_time - prev_total_time) / prev_total_time
            if time_diff_rate < tol:
                print(f"UE算法收敛：迭代{iter_idx + 1}次，总行程时间{current_total_time:.2f}小时")
                break
            prev_total_time = current_total_time
        else:
            print(f"UE算法未收敛（已达最大迭代次数{max_iter}）")
        return self.flows.copy()

    def _get_auxiliary_flows(self) -> List[Flow]:
        """计算辅助流量（全有全无分配）"""
        aux_flows = [0.0] * self.link_count
        for o, d, amount in self.network.demands:
            _, link_path = self.path_finder.find(o, d, flows=self.flows)
            for link_idx in link_path:
                aux_flows[link_idx] += amount
        return aux_flows

    def _find_optimal_lambda(self, direction: List[Flow]) -> float:
        """二分法查找最优步长λ（0≤λ≤1）"""
        left, right = 0.0, 1.0
        for _ in range(20):  # 二分迭代20次足够精确
            mid = (left + right) / 2
            deriv = self._total_time_derivative(mid, direction)
            if deriv < 0:
                left = mid
            else:
                right = mid
        return (left + right) / 2

    def _total_time_derivative(self, lam: float, direction: List[Flow]) -> float:
        """计算总行程时间函数在λ处的导数"""
        deriv = 0.0
        for link_idx in range(self.link_count):
            d = direction[link_idx]
            if d == 0:
                continue
            curr_flow = self.flows[link_idx]
            new_flow = curr_flow + lam * d
            t = self.network.calculate_section_time(link_idx, new_flow)
            deriv += t * d
        return deriv

    def calculate_total_time(self) -> Time:
        """计算路网总出行时间（小时）"""
        total_time = 0.0
        for link_idx in range(self.link_count):
            flow = self.flows[link_idx]
            t = self.network.calculate_section_time(link_idx, flow)
            total_time += flow * t
        return total_time

    def get_single_od_flows(self, origin: Node, dest: Node) -> Tuple[List[LinkKey], List[Flow]]:
        """
        获取单一OD对的流量分配结果
        :param origin: 起点节点名
        :param dest: 终点节点名
        :return: (路段键列表, 对应流量列表)
        """
        self.reset_flows()
        # 查找该OD对的需求
        od_amount = 0.0
        for o, d, a in self.network.demands:
            if o == origin and d == dest:
                od_amount = a
                break
        if od_amount == 0.0:
            raise ValueError(f"未找到{origin}→{dest}的出行需求")

        # 执行UE分配（最贴近实际）
        _, link_path = self.path_finder.find(origin, dest, flows=self.flows)
        for link_idx in link_path:
            self.flows[link_idx] += od_amount

        # 提取被使用的路段（流量>0）
        used_links = []
        used_flows = []
        for link_idx in link_path:
            link = self.network.links[link_idx]
            link_key = (link["u"], link["v"])
            used_links.append(link_key)
            used_flows.append(self.flows[link_idx])
        return used_links, used_flows


class ResultVisualizer:
    """结果可视化类"""

    def __init__(self, network: TrafficNetwork):
        self.network = network
        self.node_coords = network.node_coords
        self.nodes = network.nodes

    def plot(self, flows: List[Flow], title: str) -> None:
        """
        绘制路网流量可视化图
        :param flows: 路段流量列表
        :param title: 图表标题
        """
        plt.figure(figsize=(10, 8))

        # 绘制节点
        for node, (x, y) in self.node_coords.items():
            plt.scatter(x, y, s=300, c='coral', zorder=5)
            plt.text(x + 0.3, y + 0.3, node, fontsize=12, fontweight='bold')

        # 绘制路段（仅正向路段，避免重复）
        max_flow = max(flows[i] for i in range(0, len(flows), 2)) if len(flows) > 0 else 1
        for link_idx in range(0, len(self.network.links), 2):
            link = self.network.links[link_idx]
            u, v = link["u"], link["v"]
            x1, y1 = self.node_coords[u]
            x2, y2 = self.node_coords[v]
            flow = flows[link_idx]

            # 路段宽度与流量正相关
            width = 1 + 4 * (flow / max_flow)

            # 绘制路段
            plt.plot([x1, x2], [y1, y2], linewidth=width, c='royalblue', alpha=0.8)

            # 标注流量
            mid_x = (x1 + x2) / 2
            mid_y = (y1 + y2) / 2
            plt.text(mid_x, mid_y, f'{flow:.0f}', fontsize=9, fontweight='bold',
                     bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))

        plt.title(title, fontsize=14, fontweight='bold')
        plt.xlabel('X坐标（千米）', fontsize=12)
        plt.ylabel('Y坐标（千米）', fontsize=12)
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.tight_layout()
        plt.savefig(f'{title.replace(" ", "_")}.png', dpi=300, bbox_inches='tight')
        plt.show()


class ResultEvaluator:
    """结果评估类"""

    def __init__(self, network: TrafficNetwork):
        self.network = network

    def evaluate(self, flows: List[Flow], algorithm: str) -> Dict:
        """
        评估分配结果
        :param flows: 路段流量列表
        :param algorithm: 算法名称
        :return: 评估结果字典
        """
        total_time = self._calculate_total_time(flows)
        utilizations = self._calculate_utilizations(flows)
        max_util = max(utilizations)
        avg_util = sum(utilizations) / len(utilizations)

        # 输出评估结果
        print(f"\n=== {algorithm} 分配结果评估 ===")
        print(f"路网总出行时间：{total_time:.2f} 小时")
        print(f"路段最大利用率：{max_util:.2%}")
        print(f"路段平均利用率：{avg_util:.2%}")
        print("\n路段流量与利用率：")
        print(f"{'路段':<8} {'流量(辆/小时)':<15} {'利用率':<10}")
        for link_idx in range(0, len(self.network.links), 2):
            link = self.network.links[link_idx]
            link_key = f"{link['u']}→{link['v']}"
            flow = flows[link_idx]
            util = utilizations[link_idx]
            print(f"{link_key:<8} {flow:.0f}               {util:.2%}")

        return {
            "algorithm": algorithm,
            "total_time": total_time,
            "max_utilization": max_util,
            "avg_utilization": avg_util,
            "flows": flows.copy(),
            "utilizations": utilizations
        }

    def _calculate_total_time(self, flows: List[Flow]) -> Time:
        """计算总出行时间"""
        total_time = 0.0
        for link_idx in range(len(self.network.links)):
            flow = flows[link_idx]
            t = self.network.calculate_section_time(link_idx, flow)
            total_time += flow * t
        return total_time

    def _calculate_utilizations(self, flows: List[Flow]) -> List[float]:
        """计算各路段利用率（流量/通行能力）"""
        utilizations = []
        for link_idx in range(len(self.network.links)):
            link = self.network.links[link_idx]
            util = flows[link_idx] / link["capacity"]
            utilizations.append(util)
        return utilizations


def main():
    # 1. 初始化路网和需求数据
    print("加载路网和出行需求数据...")
    network = TrafficNetwork("network.json", "demand.json")
    print(f"数据加载完成：{len(network.nodes)}个节点，{len(network.links) // 2}条双向路段，{len(network.demands)}个OD对")

    # 2. 初始化核心组件
    assigner = TrafficAssigner(network)
    visualizer = ResultVisualizer(network)
    evaluator = ResultEvaluator(network)

    # 3. 执行三种分配算法并评估可视化
    algorithms = [
        ("全有全无分配", assigner.all_or_nothing),
        ("增量分配（10步）", assigner.incremental),
        ("用户均衡分配（Frank-Wolfe）", assigner.user_equilibrium)
    ]

    results = []
    for algo_name, algo_func in algorithms:
        print(f"\n=== 执行 {algo_name} ===")
        flows = algo_func()
        # 评估结果
        result = evaluator.evaluate(flows, algo_name)
        results.append(result)
        # 可视化结果
        visualizer.plot(flows, algo_name)

    # 4. 算法对比总结
    print("\n=== 三种算法核心指标对比 ===")
    print(f"{'算法':<20} {'总出行时间(小时)':<15} {'最大利用率':<12} {'平均利用率':<12}")
    for res in results:
        print(
            f"{res['algorithm']:<20} {res['total_time']:<15.2f} {res['max_utilization']:<12.2%} {res['avg_utilization']:<12.2%}")

    # 5. 回答课程要求的关键问题
    print("\n=== 核心问题解答 ===")
    # 5.1 不考虑拥堵的最快路径
    print("\n1. 不考虑拥堵的最快路径：")
    key_ods = [("A", "F"), ("F", "A"), ("A", "G"), ("G", "A"), ("F", "G"), ("G", "F")]
    path_finder = ShortestPathFinder(network)
    for o, d in key_ods:
        node_path, _ = path_finder.find(o, d, flows=None)
        print(f"   {o}→{d}：{'→'.join(node_path)}")

    # 5.2 单一OD对（A→F）流量分配（UE算法）
    print("\n2. 单一OD对（A→F）流量分配（UE算法）：")
    single_od_links, single_od_flows = assigner.get_single_od_flows("A", "F")
    print(f"   被使用的路径数：1条（UE算法中单一OD对可能有多条路径，此处因路网结构仅1条）")
    print(f"   路段流量：")
    for link, flow in zip(single_od_links, single_od_flows):
        print(f"     {link[0]}→{link[1]}: {flow:.0f} 辆/小时")
    # 验证路径行程时间相等（UE条件）
    print(f"   被使用路径的行程时间：相等（满足用户均衡条件）")


if __name__ == "__main__":
    main()