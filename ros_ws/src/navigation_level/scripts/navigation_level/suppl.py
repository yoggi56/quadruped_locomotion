import configparser
import os

def read_config():
    ll_hl_config = configparser.ConfigParser()
    ll_hl_config.sections()

    config_path = os.path.dirname(__file__)
    config_path += "/../../../configs/NL_LL_Config.ini"

    ll_hl_config.read(config_path)
    freq = float(ll_hl_config['DATA_TRANSFER']['Frequency'])

    return freq