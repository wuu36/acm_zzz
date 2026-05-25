import os
import user_script_main
import subprocess


TEMPLATE_CONTENT_FRONT = '''// This header file is automatically generated. Any modification to this file will get lost.\n#ifndef SUPER_CONFIG_H\n#define SUPER_CONFIG_H\n#include "typedef.h"\n\n'''
TEMPLATE_CONTENT_END = '''\nextern ST_D_SIM d_sim;\nvoid init_d_sim();\n#endif // SUPER_CONFIG_H\n'''


class SuperConfig:
    '''
    usage:
    acm = SuperConfig(motor_name)
    acm.update_super_config(d_sim, user_config)
    acm.run_simulation()
    '''

    def __init__(self, motor_name):
        print(f"\nmotor_name = {motor_name}")
        self.motor_name = motor_name
        self.c_macros = [] # 宏的名字列表
        self.super_config_header_content_front = TEMPLATE_CONTENT_FRONT
        self.super_config_header_content = ""
        self.super_config_header_content_end = TEMPLATE_CONTENT_END
        self.super_config_C_content = ""

        self.base_path = os.path.dirname(os.path.abspath(__file__))
        self.c_path = os.path.join(self.base_path, "frameworkCodes", "c")

    def parse_d_sim(self, d_sim):
        data = {}
        for key, val in d_sim.items():
            try:
                parent_node, child_node = key.split('.')
            except ValueError as e:
                print(f'yaml[simulation]里的字符串{key}要用点分离')
                print(f'yaml[simulation]里的字符串{key}要用点分离')
                print(f'yaml[simulation]里的字符串{key}要用点分离')
                raise e
            if parent_node not in data:
                data[parent_node] = {}
            data[parent_node][child_node] = val
        return data
    
    def super_config_generator(self, data, user_extend_settings=None):
        # print(f'[super_config.py] {data}')
        
        for parent_key, val in data.items():
            self.super_config_header_content += f"\ntypedef struct {{\n"
            for child_key, child_val in val.items():
                if isinstance(child_val, bool):
                    self.super_config_header_content += f"    BOOL {child_key};\n"
                elif isinstance(child_val, (int)):
                    self.super_config_header_content += f"    long {child_key};\n"
                else:
                    self.super_config_header_content += f"    REAL {child_key};\n"
            self.super_config_header_content += f"}} ST_{parent_key};\n"
        self.super_config_header_content += f"\n\ntypedef struct {{\n"
        for key in data.keys():
            self.super_config_header_content += f"    ST_{key} {key};\n"
        self.super_config_header_content += f"}} ST_D_SIM;\n"

        self.super_config_C_content = "// This c file is automatically generated. Any modification to this file will get lost.\n#include \"super_config.h\"\n#include <stdio.h>\n\n"
        self.super_config_C_content += "void init_d_sim() {\n"
        for parent_key, val in data.items():
            for child_key, child_val in val.items():
                if child_val == True or child_val == False or child_val == 'True' or child_val == 'False':
                    self.super_config_C_content += f"    d_sim.{parent_key}.{child_key} = {str(child_val).upper()};\n"
                else:
                    self.super_config_C_content += f"    d_sim.{parent_key}.{child_key} = {child_val};\n"
            self.super_config_C_content += "\n"
        self.super_config_C_content += "}\n"

        self.super_config_C_content, self.super_config_header_content = user_script_main.user_bezier_super_config(self.motor_name, data, user_extend_settings, self.super_config_C_content, self.super_config_header_content)

        # print(self.super_config_header_content)
        # print(self.super_config_C_content)
        # print(f"\n user_extend_setting: {user_extend_settings}")
    
    def data_file_generate_format(self, data_details):
        data_format_str = "\n\n\n#define DATA_FORMAT \""
        data_labels_str = "#define DATA_LABELS \""
        data_details_str = "#define DATA_DETAILS "
        for data_detail in data_details:
            data_format_str += "%g,"
            data_labels_str += str(data_detail)+","
            data_details_str += str(data_detail)+","
        data_format_str = data_format_str[:-1] + "\\n\"\n"
        data_labels_str = data_labels_str[:-1] + "\\n\"\n"
        data_details_str = data_details_str[:-1] + "\n\n\n"
        return data_format_str + data_labels_str + data_details_str

    def update_super_config(self, d_sim, user_config, user_extend_settings=None):
        self.super_config_header_content = ""
        self.d_sim_as_data = self.parse_d_sim(d_sim)
        self.super_config_generator(self.d_sim_as_data, user_extend_settings)
        

        # 生成数据文件数据标签与路径
        data_details = user_config['signal_library'] # self.load_signal_llibrary()
        data_unite = self.data_file_generate_format(data_details)
        data_file_name = "#define DATA_FILE_NAME \"../dat/" + str(self.motor_name) + ".dat\"\n"
        
        config_content = self.super_config_header_content_front + \
            f'''#define WHO_IS_USER {self.d_sim_as_data['user']['who_is_user']} \n'''\
            + self.super_config_header_content + data_unite + data_file_name + self.super_config_header_content_end
        # print(config_content)
        path_to_header = os.path.dirname(__file__) + '/frameworkCodes/c/super_config.h'
        path_to_c = os.path.dirname(__file__) + '/frameworkCodes/c/super_config.c'
        with open(path_to_header, 'w') as file:
            file.write(config_content)
        with open(path_to_c, 'w') as file:
            file.write(self.super_config_C_content)
            
    def compile_simulation(self, target="motor_simulation"):
        print("compile simulation...")
        exe_name = f"{target}.exe"
        make_cmd = ["make", "-B", exe_name]
        print(f"c_path: {self.c_path}")

        try:
            result = subprocess.run(
                make_cmd,
                cwd=self.c_path,
                text=True,
                timeout=60
            )
            if result.returncode !=0:
                print(f"compilation error:\n{result.stderr}")
                return False
            print(f"compilation successful")
            return True
        except subprocess.TimeoutExpired:
            print("compilation timeout")
            return False
        except FileNotFoundError:
            # make 不可用， 尝试...
            print("make not found, trying...")
            return False
    
    def run_simulation(self, target="motor_simulation"):
        print("run simulation...")
        exe_name = f"{target}.exe"
        exe_path = os.path.join(self.c_path, exe_name)
        
        if not os.path.exists(exe_path):
            print(f"executable not found: {exe_path}")
            return False
        
        try:
            result = subprocess.run(
                [exe_path],
                cwd=self.c_path,
                capture_output=True,
                text=True,
                timeout=120
            )
            print(result.stdout)
            if result.stderr:
                print(f"stderr: {result.stderr}")
            return True
        except subprocess.TimeoutExpired:
            print("simulation timeout")
            return False