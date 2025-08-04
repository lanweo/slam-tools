#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import argparse
from collections import deque
import time
class GraphTracer(Node):
    def __init__(self, start_topic):
        super().__init__('graph_tracer')
        print("Initializing ROS2 Graph Tracer...")
        self.start_topic = start_topic
        self.nodes_info = {}       # {node_name: {'publishes': set(), 'subscribes': set()}}
        self.topics_info = {}      # {topic_name: {'publishers': set(), 'subscribers': set()}}
        self.visited_nodes = set()
        self.topic_queue = deque([start_topic])
        
        # 设置定时器处理拓扑发现（避免在回调中直接调用服务）
        self.timer = self.create_timer(0.5, self.process_queue)
        self.get_logger().info(f"Starting trace from topic: {start_topic}")

    def get_publishers(self, topic_name):
        """获取发布指定话题的所有节点"""
        publishers = []
        node_names = self.get_node_names_and_namespaces()
        for name, namespace in node_names:
            full_name = namespace + ('' if namespace.endswith('/') else '/') + name
            try:
                pubs = self.get_publisher_names_and_types_by_node(name, namespace)
                for pub_topic, _ in pubs:
                    if pub_topic == topic_name:
                        publishers.append(full_name)
            except Exception as e:
                self.get_logger().warn(f"Error querying publishers for {full_name}: {str(e)}")
        return publishers

    def get_subscriptions(self, node_full_name):
        """获取指定节点订阅的所有话题"""
        if '/' not in node_full_name:
            return []
        
        namespace, node_name = node_full_name.rsplit('/', 1)
        namespace = namespace or '/'
        try:
            subs = self.get_subscriber_names_and_types_by_node(node_name, namespace)
            return [topic for topic, _ in subs]
        except Exception as e:
            self.get_logger().warn(f"Error querying subscriptions for {node_full_name}: {str(e)}")
            return []

    def process_queue(self):
        """处理待探索的话题队列"""
        if not self.topic_queue:
            self.timer.cancel()
            self.print_results()
            self.get_logger().info("Trace completed. Shutting down...")
            rclpy.try_shutdown()
            return

        current_topic = self.topic_queue.popleft()
        self.get_logger().debug(f"Processing topic: {current_topic}")
        
        # 获取发布当前话题的所有节点
        publishers = self.get_publishers(current_topic)
        if not publishers:
            self.topics_info.setdefault(current_topic, {'publishers': set(), 'subscribers': set()})
            self.get_logger().warn(f"No publishers found for topic: {current_topic}")
            return

        # 初始化话题信息
        topic_info = self.topics_info.setdefault(current_topic, {'publishers': set(), 'subscribers': set()})
        
        for pub_node in publishers:
            # 更新话题发布者信息
            topic_info['publishers'].add(pub_node)
            
            # 初始化节点信息
            node_info = self.nodes_info.setdefault(pub_node, {'publishes': set(), 'subscribes': set()})
            node_info['publishes'].add(current_topic)
            
            # 如果节点未访问过，获取其订阅的话题
            if pub_node not in self.visited_nodes:
                self.visited_nodes.add(pub_node)
                subscribed_topics = self.get_subscriptions(pub_node)
                
                for sub_topic in subscribed_topics:
                    # 更新节点订阅信息
                    node_info['subscribes'].add(sub_topic)
                    
                    # 更新话题订阅者信息
                    sub_topic_info = self.topics_info.setdefault(sub_topic, {'publishers': set(), 'subscribers': set()})
                    sub_topic_info['subscribers'].add(pub_node)
                    
                    # 将新发现的话题加入队列
                    if sub_topic not in self.topic_queue:
                        self.topic_queue.append(sub_topic)
                        self.get_logger().debug(f"Added new topic to queue: {sub_topic}")

    def print_results(self):
        """打印追踪结果"""
        print("\n" + "="*50)
        print("ROS2 TOPOLOGY TRACE RESULTS")
        print(f"Start Topic: {self.start_topic}")
        print("="*50 + "\n")
        
        # 打印节点信息
        print("NODE RELATIONSHIPS:")
        for node, info in self.nodes_info.items():
            pubs = ', '.join(info['publishes']) if info['publishes'] else 'None'
            subs = ', '.join(info['subscribes']) if info['subscribes'] else 'None'
            print(f"- Node: {node}")
            print(f"  Publishes: {pubs}")
            print(f"  Subscribes: {subs}")
            print()

        # 打印话题信息
        print("\nTOPIC RELATIONSHIPS:")
        for topic, info in self.topics_info.items():
            pubs = ', '.join(info['publishers']) if info['publishers'] else 'None'
            subs = ', '.join(info['subscribers']) if info['subscribers'] else 'None'
            print(f"- Topic: {topic}")
            print(f"  Publishers: {pubs}")
            print(f"  Subscribers: {subs}")
            print()

def main():
    parser = argparse.ArgumentParser(description='Trace ROS2 topic dependencies up to source nodes.')
    parser.add_argument('topic', type=str, help='Starting ROS2 topic name (e.g. /sensor/data)')
    args = parser.parse_args()
    
    rclpy.init()
    executor = SingleThreadedExecutor()
    tracer = GraphTracer(args.topic)
    executor.add_node(tracer)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    # finally:
    #     executor.shutdown()
    #     tracer.destroy_node()
    #     rclpy.shutdown()

if __name__ == '__main__':
    main()