"""
示例插件

演示插件的基本结构
"""

def main(d_sim, user_config):
    """
    插件主入口函数

    Arg:
        d_sim: 仿真数据(DataFrame)
        user_config: 用户配置(dict)
    
    Returns:
        dict: 插件结果
    """
    print("=" * 60)
    print("  Example Plugin Running")
    print("=" * 60)

    if d_sim is None:
        print("  No simulation data availabe")
        return {'status': 'no_data'}
    
    print(f"  Data points: {len(d_sim)}")