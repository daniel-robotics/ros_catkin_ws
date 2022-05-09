import argparse
import distutils.util
import os.path
import sys
import json
import rospy
import rospkg


def get_package_path(package_name):
    rospack = rospkg.RosPack()
    return rospack.get_path(package_name)

# 
def parse_args():
    ros_args = {}
    user_args = {}
    user_args_list = rospy.myargv()[1:]

    for arg in sys.argv:
        if arg.startswith("__"):
            tokens = arg.split(":=")
            ros_args[tokens[0]] = tokens[1]
    
    return ros_args, user_args

# Returns a dict containing the contents of a json file
def read_json_file(filename):
    params_dict = {}
    f = open(filename, 'r')
    params_dict = json.load(f)
    f.close()
    return params_dict


# Returns an ArgumentParser which tokenizes input arguments according to the (top-level) keys of a JSON file
def argparser_from_json(default_params_file, description=""):
    filename = default_params_file
    for idx, param in enumerate(sys.argv):
        print(param+" "+str(idx))
        if param == "--params_file" and idx+1 < len(sys.argv) and os.path.exists(sys.argv[idx+1]):
            filename = sys.argv[idx+1]
            break

    params_dict = read_json_file(filename)
    parser = argparse.ArgumentParser(description=description,
                                     epilog="Defaults loaded from "+filename,
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter,
                                     exit_on_error=False)
    parser.add_argument("--params_file", dest="params_file", default=default_params_file, type=str, help="JSON file to load default values")
    for key in params_dict:
        val = params_dict[key]
        if isinstance(val, str):
            parser.add_argument("--"+key, dest=key, default=val, type=str, help=" ")
        elif isinstance(val, bool):
            parser.add_argument("--"+key, dest=key, default=val, type=lambda x:bool(distutils.util.strtobool(x)), help=" ")
        elif isinstance(val, int):
            parser.add_argument("--"+key, dest=key, default=val, type=int, help=" ")
        elif isinstance(val, float):
            parser.add_argument("--"+key, dest=key, default=val, type=float, help=" ")
        elif val is None:
            parser.add_argument("--"+key, dest=key, default=None, help=" ")
    return parser


