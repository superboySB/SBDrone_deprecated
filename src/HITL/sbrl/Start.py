import os
import json
import os
import pathlib
import time
import Utils
import subprocess
import atexit

homeDir = str(pathlib.Path.home())
projectName = Utils.getConfig()['projectName']
envProcesses = int(Utils.getConfig()['envProcesses'])
storage_port = int(Utils.getConfig()['storage_port'])
headless = bool(Utils.getConfig()['headless'])

def changeUEIPJson(port):
    with open(str(pathlib.Path.home()) + "/Documents/AirSim/settings.json", "r") as jsonFile:
        data = json.load(jsonFile)

    data["ApiServerPort"] = port

    with open(str(pathlib.Path.home()) + "/Documents/AirSim/settings.json", "w") as jsonFile:
        json.dump(data, jsonFile, indent=4)


# os.system('gnome-terminal -- python Storage.py {}'.format(storage_port))
# time.sleep(5)
# os.system('gnome-terminal -- python Trainer.py {}'.format(storage_port))


storage_procress = subprocess.Popen(['python3','Storage.py',f"{storage_port}"],shell=False, bufsize=0)
atexit.register(storage_procress.terminate)
time.sleep(5)

trainer_procress = subprocess.Popen(['python3','Trainer.py',f"{storage_port}"],shell=False,bufsize=0)
atexit.register(trainer_procress.terminate)


for i in range(envProcesses):
    port = storage_port + i + 1
    changeUEIPJson(port)
    if headless:
        # os.system('gnome-terminal -- ./UEBinary/{projectName}.sh -RenderOffscreen -windowed -NoVSync'.format(projectName=projectName))
        ue_procress = subprocess.Popen([f'./UEBinary/{projectName}.sh','-RenderOffscreen','-windowed','-NoVSync'],shell=False,bufsize=0)
        atexit.register(ue_procress.terminate)
    else:
        windowX = 1000 * i
        windowY = 1000
        # os.system('gnome-terminal -- ./UEBinary/{projectName}.sh -windowed -WinX={WinX} -WinY={WinY} -NoVSync'.format(
        #     projectName=projectName,
        #     WinX=windowX,
        #     WinY=windowY))
        ue_procress = subprocess.Popen([f'./UEBinary/{projectName}.sh','--windowed',f'-WinX={windowX}',f'-WinY={windowY}','-NoVSync'],shell=False,bufsize=0)
        atexit.register(ue_procress.terminate)
    time.sleep(4)

time.sleep(5)
for i in range(envProcesses):
    UE_port = storage_port + i + 1
    UE_Address = "127.0.0.1"
    # os.system('gnome-terminal -- python PyClient.py {UE_port} {UE_Address} {storage_port}'.format(UE_port=UE_port, UE_Address="127.0.0.1", storage_port=storage_port))
    agent_procress = subprocess.Popen(['python3','PyClient.py',f'{UE_port}',f'{UE_Address}',f'{storage_port}'],shell=False, bufsize=0)
    atexit.register(agent_procress.terminate)