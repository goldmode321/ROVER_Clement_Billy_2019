# scp -r ../RL_One_Foot/ nvidia@tx2-lab931.local:/home/nvidia/onefoot_project
import os
import sys
address = {
    # 's2' : [51571, "nchulab931.ddns.net", "rudy3742"],
    # 'tx2': [22, "nvidia@TX2-Lab931.local", "nvidia"],
    # 'lab': [51532, "nchulab931.ddns.net", "rudy3742"],
    # 'cle': [22, "clement@192.168.1.101", "clement"],
    # 'nano': [51521, "nchulab931.ddns.net", "rudy3742"],
    # 'cle': [51532, "clement@nchulab931.ddns.net", "clement"]
    'rover': [22, "pi@192.168.5.2", "pi"]
}
try:
    info = address['rover']
except IndexError:
    print("Please specify Server...")
    sys.exit()

cmds = [
    "scp -i ~/.ssh/rover_rsa -P {0} ./*.py {1}:/home/{2}/ROVER/".format(info[0], info[1], info[2]),
    # "scp -P {0} ./UI/*UI.py {1}:/home/{2}/onefoot_project/RL_One_Foot/UI/".format(info[0], info[1], info[2])
]
for cmd in cmds:
    os.system(cmd)
