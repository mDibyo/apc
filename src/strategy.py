import os.path as osp
import json
import numpy as np
from utils import JSON_DIR, OBJ_LIST, obj_ease


def parse_json(name):
    order = json.load(open(osp.join(JSON_DIR, name)))
    q = []
    targets = {}
    for target in order["work_order"]:
        q.append(target["bin"])
        targets[target["bin"]] = target["item"]

    def priority(bin_name):
        features = [len(order["bin_contents"][bin_name]),
                    obj_ease[targets[bin_name]],
                    sum([1 if b in bin_name else 0 for b in ["A", "B", "C"]]) ]
        return sum([10**i * f for i,f in enumerate(features)])
    
    seq = sorted(q, key=priority)
    return seq, targets, order["bin_contents"]
    
s,t,b  = parse_json("apc.json")
