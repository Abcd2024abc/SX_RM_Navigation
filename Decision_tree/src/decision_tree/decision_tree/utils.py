def ns_topic(node, topic):
    """根据命名空间拼接话题或服务名"""
    node_ns = node.get_namespace()
    
    if node_ns.startswith('/'):
        node_ns = node_ns[1:]
    
    if topic.startswith('/'):
        topic = topic[1:]
        
    if node_ns:
        return f"/{node_ns}/{topic}"
    else:
        return f"/{topic}"