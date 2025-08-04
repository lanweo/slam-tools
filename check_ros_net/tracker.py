#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import argparse
from collections import deque
import time
import os
import xml.etree.ElementTree as ET
from xml.dom import minidom
import hashlib

class GraphTracer(Node):
    def __init__(self, start_topic):
        super().__init__('graph_tracer')
        print("Initializing ROS2 Graph Tracer...")
        self.start_topic = start_topic
        self.nodes_info = {}       # {node_name: {'publishes': set(), 'subscribes': set()}}
        self.topics_info = {}      # {topic_name: {'publishers': set(), 'subscribers': set()}}
        self.visited_nodes = set()
        self.topic_queue = deque([start_topic])
        
        # 增强过滤系统
        self.filtered_topics = {
            '/parameter_events',  # 显式过滤参数事件话题
            '/rosout',
            '/clock',
            '/tf',
            '/tf_static',
        }
        self.filtered_node_prefixes = ['_ros2cli_daemon']  # 过滤特定节点前缀
        
        self.timer = self.create_timer(0.5, self.process_queue)
        self.get_logger().info(f"Starting trace from topic: {start_topic}")

    def get_publishers(self, topic_name):
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
        if not self.topic_queue:
            self.timer.cancel()
            self.print_results()
            self.generate_drawio_file()
            self.get_logger().info("Trace completed. Shutting down...")
            rclpy.try_shutdown()
            return

        current_topic = self.topic_queue.popleft()
        self.get_logger().debug(f"Processing topic: {current_topic}")
        
        # 增强过滤逻辑：同时检查话题和节点
        for filtered_topic in self.filtered_topics:
            if current_topic == filtered_topic:
                self.get_logger().info(f"Skipping filtered topic: {current_topic}")
                return
        # if current_topic in self.filtered_topics:
        #     self.get_logger().info(f"Skipping filtered topic: {current_topic}")
        #     return
            
        publishers = self.get_publishers(current_topic)
        if not publishers:
            self.topics_info.setdefault(current_topic, {'publishers': set(), 'subscribers': set()})
            self.get_logger().warn(f"No publishers found for topic: {current_topic}")
            return

        topic_info = self.topics_info.setdefault(current_topic, {'publishers': set(), 'subscribers': set()})
        
        for pub_node in publishers:
            # 检查节点是否在过滤前缀列表中
            skip_node = any(pub_node.split('/')[-1].startswith(prefix) for prefix in self.filtered_node_prefixes)
            if skip_node and current_topic in self.filtered_topics:
                self.get_logger().info(f"Skipping filtered node: {pub_node} for topic {current_topic}")
                continue
                
            topic_info['publishers'].add(pub_node)
            node_info = self.nodes_info.setdefault(pub_node, {'publishes': set(), 'subscribes': set()})
            node_info['publishes'].add(current_topic)
            
            if pub_node not in self.visited_nodes:
                self.visited_nodes.add(pub_node)
                subscribed_topics = self.get_subscriptions(pub_node)
                
                for sub_topic in subscribed_topics:
                    if sub_topic in self.filtered_topics:
                        continue
                    node_info['subscribes'].add(sub_topic)
                    sub_topic_info = self.topics_info.setdefault(sub_topic, {'publishers': set(), 'subscribers': set()})
                    sub_topic_info['subscribers'].add(pub_node)
                    
                    if sub_topic not in self.topic_queue:
                        self.topic_queue.append(sub_topic)
                        self.get_logger().debug(f"Added new topic to queue: {sub_topic}")

    def print_results(self):
        print("\n" + "="*50)
        print("ROS2 TOPOLOGY TRACE RESULTS")
        print(f"Start Topic: {self.start_topic}")
        print(f"Filtered Topics: {', '.join(self.filtered_topics)}")
        print(f"Filtered Node Prefixes: {', '.join(self.filtered_node_prefixes)}")
        print("="*50 + "\n")
        
        print("NODE RELATIONSHIPS:")
        for node, info in self.nodes_info.items():
            # 动态过滤逻辑
            filtered_publishes = [
                t for t in info['publishes'] 
                if t not in self.filtered_topics and 
                not (any(node.split('/')[-1].startswith(prefix) for prefix in self.filtered_node_prefixes) and t == '/parameter_events')
            ]
            filtered_subscribes = [t for t in info['subscribes'] if t not in self.filtered_topics]
            
            # 如果节点完全被过滤则跳过
            if not filtered_publishes and not filtered_subscribes:
                continue
                
            pubs = ', '.join(filtered_publishes) if filtered_publishes else 'None'
            subs = ', '.join(filtered_subscribes) if filtered_subscribes else 'None'
            
            print(f"- Node: {node}")
            print(f"  Publishes: {pubs}")
            print(f"  Subscribes: {subs}")
            print()

        print("\nTOPIC RELATIONSHIPS (Filtered):")
        for topic, info in self.topics_info.items():
            if topic in self.filtered_topics:
                continue
                
            # 应用节点过滤
            valid_publishers = [
                p for p in info['publishers']
                if not (any(p.split('/')[-1].startswith(prefix) for prefix in self.filtered_node_prefixes) and topic == '/parameter_events')
            ]
            valid_subscribers = [s for s in info['subscribers'] ]
            
            if not valid_publishers and not valid_subscribers:
                continue
                
            pubs = ', '.join(valid_publishers) if valid_publishers else 'None'
            subs = ', '.join(valid_subscribers) if valid_subscribers else 'None'
            print(f"- Topic: {topic}")
            print(f"  Publishers: {pubs}")
            print(f"  Subscribers: {subs}")
            print()

    def generate_drawio_file(self):
        try:
            root = ET.Element("mxfile", host="app.diagrams.net")
            diagram = ET.SubElement(root, "diagram", name="ROS2 Graph", id="diagram_1")
            mx_graph_model = ET.SubElement(diagram, "mxGraphModel", 
                                          dx="1426", dy="836", grid="1", gridSize="10", 
                                          guides="1", tooltips="1", connect="1", 
                                          arrows="1", fold="1", page="1", pageScale="1", 
                                          pageWidth="850", pageHeight="1100", math="0", 
                                          shadow="0")
            
            root_elem = ET.SubElement(mx_graph_model, "root")
            ET.SubElement(root_elem, "mxCell", id="0")
            ET.SubElement(root_elem, "mxCell", id="1", parent="0")
            
            node_ids = {}
            next_id = 2
            node_positions = {}
            x, y = 100, 100
            
            # 节点创建逻辑
            for node_name in self.nodes_info:
                # 动态过滤
                filtered_publishes = [
                    t for t in self.nodes_info[node_name]['publishes'] 
                    if t not in self.filtered_topics and 
                    not (any(node_name.split('/')[-1].startswith(prefix) for prefix in self.filtered_node_prefixes) and t == '/parameter_events'
                )]
                filtered_subscribes = [t for t in self.nodes_info[node_name]['subscribes'] if t not in self.filtered_topics]
                
                if not filtered_publishes and not filtered_subscribes:
                    continue
                
                node_id = str(next_id)
                node_ids[node_name] = node_id
                next_id += 1
                
                width = max(120, 10 * len(node_name))
                height = 40
                
                node_elem = ET.SubElement(root_elem, "mxCell", 
                                         id=node_id,
                                         value=node_name,
                                         style="rounded=0;whiteSpace=wrap;html=1;fillColor=#dae8fc;strokeColor=#6c8ebf;",
                                         vertex="1",
                                         parent="1")
                
                geometry = ET.SubElement(node_elem, "mxGeometry")
                geometry.set("x", str(x))
                geometry.set("y", str(y))
                geometry.set("width", str(width))
                geometry.set("height", str(height))
                geometry.set("as", "geometry")
                
                node_positions[node_name] = (x + width/2, y + height/2)
                x += width + 100
                if x > 1000:
                    x = 100
                    y += 150

            # === 关键优化：解决边重叠问题 ===
            # 使用哈希表记录相同节点间的多条边
            edge_groups = {}
            for topic_name, topic_info in self.topics_info.items():
                if topic_name in self.filtered_topics:
                    continue
                    
                for publisher in topic_info['publishers']:
                    # 应用节点过滤
                    if (any(publisher.split('/')[-1].startswith(prefix) for prefix in self.filtered_node_prefixes)) and topic_name == '/parameter_events':
                        continue
                        
                    if publisher not in node_ids:
                        continue
                        
                    for subscriber in topic_info['subscribers']:
                        if subscriber not in node_ids:
                            continue
                            
                        # 创建边分组键
                        edge_key = (node_ids[publisher], node_ids[subscriber])
                        if edge_key not in edge_groups:
                            edge_groups[edge_key] = []
                        edge_groups[edge_key].append(topic_name)
            
            # 为每组边创建单条聚合边
            for (source_id, target_id), topics in edge_groups.items():
                edge_id = str(next_id)
                next_id += 1
                
                # 创建聚合边（显示多个话题）
                edge_value = "\n".join(topics) if len(topics) > 1 else topics[0]
                edge_elem = ET.SubElement(root_elem, "mxCell", 
                                        id=edge_id,
                                        value=edge_value,
                                        style=("endArrow=classic;html=1;rounded=0;strokeColor=#82b366;"
                                              "fontColor=#000000;align=center;verticalAlign=bottom;"),
                                        edge="1",
                                        parent="1",
                                        source=source_id,
                                        target=target_id)
                
                geometry = ET.SubElement(edge_elem, "mxGeometry")
                geometry.set("relative", "1")
                geometry.set("as", "geometry")
                
                # 添加弯曲点避免完全重叠
                if len(topics) > 1:
                    mxPoint = ET.SubElement(geometry, "mxPoint")
                    mxPoint.set("x", "20")
                    mxPoint.set("y", "-10")
                    mxPoint.set("as", "offset")
            # === 结束优化 ===
            
            xml_str = ET.tostring(root, encoding="utf-8")
            dom = minidom.parseString(xml_str)
            pretty_xml = dom.toprettyxml(indent="  ")
            
            output_file = f"ros2_graph_{self.start_topic.replace('/', '_')}.drawio"
            with open(output_file, "w") as f:
                f.write(pretty_xml)
            
            print(f"\nDraw.io file generated: {output_file}")
            print("1. Open draw.io (https://app.diagrams.net/)")
            print("2. Click 'File' > 'Import From' > 'Device...'")
            print(f"3. Select the file: {output_file}")
            print("4. Use 'Layout' > 'Arrange' > 'Organic' for best results")
            print("5. Grouped edges show multiple topics separated by newlines")
            
        except Exception as e:
            self.get_logger().error(f"Failed to generate draw.io file: {str(e)}")

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

if __name__ == '__main__':
    main()