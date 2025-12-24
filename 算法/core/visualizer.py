import matplotlib.pyplot as plt
import numpy as np


class ResultVisualizer:
    def __init__(self, nodes, links, link_flows, link_id_map):
        self.nodes = nodes  # 节点字典：{节点名: (x, y)}
        self.links = links  # 路段列表
        self.link_flows = link_flows  # 路段流量
        self.link_id_map = link_id_map  # 路段ID映射
        self.node_names = list(nodes.keys())
        self.node_coords = np.array([nodes[name] for name in self.node_names])

    def plot_network(self, algorithm_name):
        """绘制路网及路段流量"""
        plt.figure(figsize=(12, 8))

        # 绘制节点
        x = self.node_coords[:, 0]
        y = self.node_coords[:, 1]
        plt.scatter(x, y, s=200, c='orange', zorder=5)
        for i, name in enumerate(self.node_names):
            plt.text(x[i] + 0.5, y[i] + 0.5, name, fontsize=12, fontweight='bold')

        # 绘制路段（正向路段，避免重复）
        max_flow = max(self.link_flows[i] for i in range(0, len(self.link_flows), 2))
        for link_idx in range(0, len(self.links), 2):
            u = self.links[link_idx][0]
            v = self.links[link_idx][1]
            u_name = self.node_names[u]
            v_name = self.node_names[v]
            flow = self.link_flows[link_idx]

            # 路段宽度与流量正相关（最大宽度5）
            width = 1 + 4 * (flow / max_flow) if max_flow != 0 else 1

            # 绘制路段
            plt.plot(
                [self.node_coords[u][0], self.node_coords[v][0]],
                [self.node_coords[u][1], self.node_coords[v][1]],
                linewidth=width, c='steelblue', alpha=0.7
            )

            # 标注流量（路段中点）
            mid_x = (self.node_coords[u][0] + self.node_coords[v][0]) / 2
            mid_y = (self.node_coords[u][1] + self.node_coords[v][1]) / 2
            plt.text(mid_x, mid_y, f'{flow:.0f}', fontsize=10, fontweight='bold',
                     bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        plt.title(f'{algorithm_name} 分配结果：路网流量可视化', fontsize=14, fontweight='bold')
        plt.xlabel('X坐标（千米）', fontsize=12)
        plt.ylabel('Y坐标（千米）', fontsize=12)
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.tight_layout()
        plt.savefig(f'{algorithm_name}_network_flow.png', dpi=300, bbox_inches='tight')
        plt.show()