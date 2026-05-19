


def user_pre_process(d_sim, user_config):
    # TODO: 你可以任意更改下面内容
    """
    eg:
    func1()
    func2()
    ...
    """
    return d_sim

def user_bezier_super_config(motor_name, data, user_extend_settings, super_config_C_content, super_config_header_content):
    if user_extend_settings:
        if user_extend_settings.get("bezier_moo"):
            print("bezier")
        elif user_extend_settings.get("bezier_C_save"):
            print("bezier 123")
    else:
        print("beizer 123")
    
    return super_config_C_content, super_config_header_content
    
