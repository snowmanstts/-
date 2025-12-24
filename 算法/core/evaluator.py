class ResultEvaluator:
    def __init__(self, links, link_flows):
        self.links = links  # 路段列表
        self.link_flows = link_flows  # 路段流量

    def calculate_total_time(self):
        """计算路网总出行时间（单位：小时）"""
        total_time = 0.0
        for link_idx in range(len(self.links)):
            q = self.link_flows[link_idx]
            t0 = self.links[link_idx][5]
            cap = self.links[link_idx][2]
            t = t0 * (1 + q / cap) ** 2
            total_time += q * t
        return total_time

    def calculate_link_utilization(self):
        """计算各路段利用率（流量/通行能力）"""
        utilization = []
        for link_idx in range(len(self.links)):
            q = self.link_flows[link_idx]
            cap = self.links[link_idx][2]
            util = q / cap if cap != 0 else 0.0
            utilization.append(util)
        return utilization

    def print_evaluation(self, algorithm_name):
        """打印评估结果"""
        total_time = self.calculate_total_time()
        utilization = self.calculate_link_utilization()
        max_util = max(utilization)
        avg_util = sum(utilization) / len(utilization)

        print(f"\n=== {algorithm_name} 分配结果评估 ===")
        print(f"路网总出行时间：{total_time:.2f} 小时")
        print(f"路段最大利用率：{max_util:.2%}")
        print(f"路段平均利用率：{avg_util:.2%}")

        # 打印各路段流量和利用率（仅正向路段，避免重复）
        print("\n路段流量与利用率：")
        print(f"{'路段':<6} {'流量(辆/小时)':<15} {'利用率':<10}")
        for link_idx in range(0, len(self.links), 2):  # 正向路段（步长2）
            u = self.links[link_idx][0]
            v = self.links[link_idx][1]
            node_names = list(self.links[0][0].keys())  # 从nodes字典获取节点名
            u_name = node_names[u]
            v_name = node_names[v]
            q = self.link_flows[link_idx]
            util = utilization[link_idx]
            print(f"{u_name}→{v_name:<4} {q:.0f}               {util:.2%}")
            print(f"{u_name}→{v_name:<4} {q:.0f}               {util:.2%}")