#!/usr/bin/python3

import json, argparse, os, sys, re

def convert_feet_meters(val):
    return (str(val)+' feet', str(val/0.3048)+' meters')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Loads, parses and converts mission json data into seperate human readable files')
    parser.add_argument('mission_data', help='json file that holds auvsi mission data', default="mission_data.json")
    args = parser.parse_args()
    mission_data = args.mission_data
    
    if not mission_data.endswith(".json"):
        raise ValueError("Not a Json file type")

    with open(mission_data, 'r') as f:
        mission_dict = json.load(f)

    for title_key, title_val in mission_dict.items():
        if type(title_val) == list:
            for item in title_val:
                for key, val in item.items():
                    if re.search(key, 'atlitude') or key in ['radius', 'height']:
                        item[key] = convert_feet_meters(val)
        elif type(title_val) == dict:
            for key, val in title_val.items():
                if re.search(key, 'atlitude') or key in ['radius', 'height']:
                    item[key] = convert_feet_meters(val)
        with open(title_key+'.txt', 'w') as title_f:
            title_f.write(title_key+'\n')
            json.dump(title_val, title_f, indent=4)
