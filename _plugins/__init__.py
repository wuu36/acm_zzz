import os
import importlib.util

# 插件目录
PLUGINS_DIR = os.path.dirname(os.path.abspath(__file__))

# 已加载的插件
_plugins = {}

def discover_plugins():
    """
    发现所有可用插件

    Returns:
        dict: {plug_name: plugin_info}
    """
    plugins = {}

    if not os.path.exists(PLUGINS_DIR):
        return plugins
    
    for item in os.listdir(PLUGINS_DIR):
        item_path = os.path.join(PLUGINS_DIR, item)

        if os.path.isdir(item_path) and item.startswith('plugin_'):
            plugin_name = item[len('plugin_'):]
            print(plugin_name)

            index_file = os.path.join(item_path, 'index.py')
            if os.path.exists(index_file):
                plugins[plugin_name] = {
                    'name': plugin_name,
                    'path': item_path,
                    'index_file': index_file
                }

    return plugins

def load_plugin(plugin_name):
    """
    加载制定插件

    Args:
        plugin_name: 插件名称

    Returns:
        module: 插件模块对象
    """
    if plugin_name in _plugins:
        return _plugins[plugin_name]

    plugins = discover_plugins()
    
    if plugin_name not in plugins:
        raise ImportError(f"Plugin '{plugin_name}' not found")
    
    plugin_info = plugins[plugin_name]
    index_file = plugin_info['index_file']
    
    # 动态加载模块
    spec = importlib.util.spec_from_file_location(
        f"_plugins.plugin_{plugin_name}",
        index_file
    )

    if spec and spec.loader:
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        _plugins[plugin_name] = module
        return module

    return None

def get_plugin_main(plugin_name):
    """
    获取插件的main函数

    Args:
        plugin_name: 插件名称

    Returns:
        function: main函数对象
    """
    plugin_module = load_plugin(plugin_name)

    if plugin_module and hasattr(plugin_module, 'main'):
        return plugin_module.main
    
    return None

def list_plugins():
    """
    列出所有可用插件

    Returns:
        list: 插件名称列表
    """
    plugins = discover_plugins()
    return list(plugins.keys())

