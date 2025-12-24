from core.data_loader import DataLoader
from core.algorithms import TrafficAssignment
from core.evaluator import ResultEvaluator
from core.visualizer import ResultVisualizer


def main():
    # 1. 加载数据
    print("加载路网和出行需求数据...")
    loader = DataLoader(network_path="network.json", demand_path="demand.json")
    nodes, links, demand, link_id_map = loader.load_data()
    print(f"数据加载完成：{len(nodes)}个节点，{len(links) // 2}条双向路段，{len(demand)}个出行需求")

    # 2. 初始化交通分配实例
    ta = TrafficAssignment(nodes, links, demand)

    # 3. 执行三种分配算法
    algorithms = {
        "全有全无分配": ta.all_or_nothing,
        "增量分配（10步）": ta.incremental,
        "用户均衡分配（Frank-Wolfe）": ta.user_equilibrium
    }

    for algo_name, algo_func in algorithms.items():
        print(f"\n=== 执行 {algo_name} ===")
        link_flows = algo_func()

        # 4. 结果评估
        evaluator = ResultEvaluator(links, link_flows)
        evaluator.print_evaluation(algo_name)

        # 5. 结果可视化
        visualizer = ResultVisualizer(nodes, links, link_flows, link_id_map)
        visualizer.plot_network(algo_name)

    # 6. 对比不同算法的总出行时间
    print("\n=== 三种算法总出行时间对比 ===")
    for algo_name, algo_func in algorithms.items():
        link_flows = algo_func()
        evaluator = ResultEvaluator(links, link_flows)
        total_time = evaluator.calculate_total_time()
        print(f"{algo_name}: {total_time:.2f} 小时")


if __name__ == "__main__":
    main()