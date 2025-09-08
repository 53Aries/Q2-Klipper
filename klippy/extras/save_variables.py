# Save arbitrary variables so that values can be kept across restarts.
#
# Copyright (C) 2020 Dushyant Ahuja <dusht.ahuja@gmail.com>
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, ast, configparser

class SaveVariables:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.filename = os.path.expanduser(config.get('filename'))
        self.allVariables = {}
        
        self.default_values = {

            # 新增 AI 摄像头设置变量
            'enable_noodle_detection': 1,  # 是否开启炒面检测 (0=关闭, 1=开启)
            'noodle_sensitivity_level': "MEDIUM",  # 炒面检测灵敏度 (LOW/MEDIUM/HIGH)
            'enable_model_shift_detection': 1,  # 是否开启模型位移检测
            'model_shift_sensitivity_level': "MEDIUM",  # 模型位移检测灵敏度
            'enable_model_collapse_detection': 1,  # 是否开启模型倒塌检测
            'model_collapse_sensitivity_level': "MEDIUM",  # 模型倒塌检测灵敏度
            'enable_platform_detection': 1,  # 是否开启平台板检测
            'enable_pre_print_model_check': 1,  # 是否开启打印前模型未清空检测
            "qdc_ai_error_code": "",
            'video_storage_path': "/home/qidi/video/",  # 默认视频保存路径
            'enable_management_ui': 0,  # 默认关闭管理后台

            'calibration_type': 0,
            'calibration_step': 0,

            'auto_reload_detect': 0, 'auto_read_rfid': 0, 'auto_init_detect': 0,
            'box_count': 0, 'enable_box': 0,
            'slot_sync': "", 'retry_step': "", 'last_load_slot': ""
        }
        
        for i in range(16):
            self.default_values[f'filament_slot{i}'] = 0
            self.default_values[f'color_slot{i}'] = 0
            self.default_values[f'vendor_slot{i}'] = 0
            self.default_values[f'slot{i}'] = 0
            self.default_values[f'value_t{i}'] = ""
        
        self.default_values['filament_slot16'] = 0
        self.default_values['color_slot16'] = 0
        self.default_values['vendor_slot16'] = 0
        
        for key, value in self.default_values.items():
            setattr(self, key, value)
            
        try:
            if not os.path.exists(self.filename):
                open(self.filename, "w").close()
            self.loadVariables()
        except self.printer.command_error as e:
            raise config.error(str(e))
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SAVE_VARIABLE', self.cmd_SAVE_VARIABLE,
                               desc=self.cmd_SAVE_VARIABLE_help)
                               
    def load_variable(self, section, option):
        varfile = configparser.ConfigParser()
        try:
            varfile.read(self.filename)
            return varfile.get(section, option)
        except:
            msg = "Unable to parse existing variable file"
            logging.exception(msg)
            raise self.printer.command_error(msg)
            
    def save_variable(self, section, option, value):
        varfile = configparser.ConfigParser()
        try:
            varfile.read(self.filename)
            if not varfile.has_section(section):
                varfile.add_section(section)
            varfile.set(section, option, value)
            with open(self.filename, 'w') as configfile:
                varfile.write(configfile)
        except Exception as e:
            msg = "Unable to save variable"
            logging.exception(msg)
            raise self.printer.command_error(msg)
        self.loadVariables()
        
    def loadVariables(self):
        allvars = {}
        varfile = configparser.ConfigParser()
        try:
            varfile.read(self.filename)
            if varfile.has_section('Variables'):
                for name, val in varfile.items('Variables'):
                    allvars[name] = ast.literal_eval(val)
        except:
            msg = "Unable to parse existing variable file"
            logging.exception(msg)
            raise self.printer.command_error(msg)
        self.allVariables = allvars
        
        for key, default in self.default_values.items():
            setattr(self, key, self.allVariables.get(key, default))
            
    cmd_SAVE_VARIABLE_help = "Save arbitrary variables to disk"
    
    def cmd_SAVE_VARIABLE(self, gcmd):
        varname = gcmd.get('VARIABLE')
        value = gcmd.get('VALUE')
        try:
            value = ast.literal_eval(value)
        except ValueError as e:
            raise gcmd.error("Unable to parse '%s' as a literal" % (value,))
        newvars = dict(self.allVariables)
        newvars[varname] = value
        # Write file
        varfile = configparser.ConfigParser()
        varfile.add_section('Variables')
        for name, val in sorted(newvars.items()):
            varfile.set('Variables', name, repr(val))
        try:
            f = open(self.filename, "w")
            varfile.write(f)
            f.close()
        except:
            msg = "Unable to save variable"
            logging.exception(msg)
            raise gcmd.error(msg)
        self.loadVariables()
        
    def get_status(self, eventtime):
        status = {'variables': self.allVariables}
        
        for key in self.default_values.keys():
            status[key] = getattr(self, key)
            
        return status

def load_config(config):
    return SaveVariables(config)