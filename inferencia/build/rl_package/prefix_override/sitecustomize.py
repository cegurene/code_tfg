import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/carlos/Escritorio/TFG/code_tfg/inferencia/install/rl_package'
