import random
from datetime import datetime
import os
import json
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
from matplotlib.patches import FancyBboxPatch


class YcbObjects:
    def __init__(self, load_path, mod_orn=None, mod_stiffness=None, exclude=None):
        self.load_path = load_path
        self.mod_orn = mod_orn
        self.mod_stiffness = mod_stiffness
        with open(load_path + '/obj_list.txt') as f:
            lines = f.readlines()
            self.obj_names = [line.rstrip('\n') for line in lines]
        if exclude is not None:
            for obj_name in exclude:
                self.obj_names.remove(obj_name)

    def shuffle_objects(self):
        random.shuffle(self.obj_names)

    def get_obj_path(self, obj_name):
        return f'{self.load_path}/Ycb{obj_name}/model.urdf'

    def check_mod_orn(self, obj_name):
        if self.mod_orn is not None and obj_name in self.mod_orn:
            return True
        return False

    def check_mod_stiffness(self, obj_name):
        if self.mod_stiffness is not None and obj_name in self.mod_stiffness:
            return True
        return False

    def get_obj_info(self, obj_name):
        return self.get_obj_path(obj_name), self.check_mod_orn(obj_name), self.check_mod_stiffness(obj_name)

    def get_n_first_obj_info(self, n):
        info = []
        for obj_name in self.obj_names[:n]:
            info.append(self.get_obj_info(obj_name))
        return info

def write_summary(path, tries, target, grasp):
    with open(path+'/summary.txt', 'w') as f:
        total_tries = sum(tries.values())
        total_target = sum(target.values())
        total_grasp = sum(grasp.values())
        f.write('Total:\n')
        f.write(
            f'Target acc={total_target/total_tries:.3f} ({total_target}/{total_tries}) Grasp acc={total_grasp/total_tries:.3f} ({total_grasp}/{total_tries})\n')
        f.write('\n')
        f.write("Accuracy per object:\n")
        for obj in tries.keys():
            n_tries = tries[obj]
            n_t = target[obj]
            n_g = grasp[obj]
            f.write(
                f'{obj}: Target acc={n_t/n_tries:.3f} ({n_t}/{n_tries}) Grasp acc={n_g/n_tries:.3f} ({n_g}/{n_tries})\n')


def summarize(path, trials):
    with open(path+'/data_tries.json') as data:
        tries = json.load(data)
    with open(path+'/data_target.json') as data:
        target = json.load(data)
    with open(path+'/data_grasp.json') as data:
        grasp = json.load(data)
    plt(path, tries, target, grasp, trials)
    write_summary(path, tries, target, grasp)