import yaml
from graphviz import Digraph
import re

def create_tf_graph(yaml_file, output_pdf):
    # 读取YAML文件
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)

    # 创建有向图
    dot = Digraph(comment='TF Frames', 
                 format='pdf',
                 graph_attr={'rankdir': 'TB', 'dpi': '300'},
                 node_attr={'shape': 'box', 'style': 'filled', 'fillcolor': '#F0F8FF'})

    # 添加所有节点和边
    for frame, info in data.items():
        # 清理frame名称中的非法字符
        clean_frame = re.sub(r'[/:]', '_', frame)
        
        # 添加节点
        label = f"{frame}\n"
        label += f"Rate: {info['rate']} Hz\n"
        label += f"Buffer: {info['buffer_length']}s"
        dot.node(clean_frame, label=label)
        
        # 添加父子关系边
        if info['parent'] and info['parent'] != 'None':
            clean_parent = re.sub(r'[/:]', '_', info['parent'])
            dot.edge(clean_parent, clean_frame)

    # 生成PDF
    dot.render(output_pdf, view=False, cleanup=True)
    print(f"PDF文件已生成：{output_pdf}.pdf")

if __name__ == "__main__":
    create_tf_graph("tf_frames.yaml", "tf_graph5")