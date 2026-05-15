"""
插件系统演示

展示插件系统如何工作
"""
import os
import sys

print("=" * 60)
print("  Plugin System Demo")
print("=" * 60)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

print("\n Importing plugin system...")
import _plugins

# 列出所有插件
print("\n" + "=" * 50)
print("Step 1: List Available Plugins")
print("=" * 50)

plugins = _plugins.list_plugins()
print(f"\n Found {len(plugins)} plugins:")
for plugin in plugins:
    print(f"    - {plugin}")

# 加载插件
print("\n" + "=" * 50)
print("Step 2: Load Plugins")
print("\n" + "=" * 50)

for plugin_name in plugins:
    print(f"\n  Loading: {plugin_name}")
    plugin_module = _plugins.load_plugin(plugin_name)
    if plugin_module:
        print(f"    Loaded: {plugin_module.__name__}")
    else:
        print(f"    Failed to load")

# 获取插件main函数
print("\n" + "=" * 50)
print("Step 3: Get Plugin Main Functions")
print("\n" + "=" * 50)

for plugin_name in plugins:
    print(f"\n  Getting main() for: {plugin_name}")
    main_func = _plugins.get_plugin_main(plugin_name)
    if main_func:
        print(f"    Got function: {main_func.__name__}")
        print(f"    Callable: {callable(main_func)}")

print("\n" + "=" * 60)
print("  Plugin System Demo Complete!")
print("=" * 60)