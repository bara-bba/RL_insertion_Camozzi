import doodad
from doodad.launch import launch_api

local = doodad.LocalMode()
gcp_mode = doodad.GCPMode(<fill in arguments>)

mnt = doodad.MountLocal(
    local_dir='foo',
    mount_point='./mymount',
    output=True
)

# This will run locally
doodad.run_command(
    command='cat ./mymount/secret.txt',
    mounts=[mnt],
    mode=local,
)

# This will run remotely
doodad.run_command(
    command='cat ./mymount/secret.txt',
    mounts=[mnt],
    mode=gcp_mode
)

launch_api.run_python('/home/bara/PycharmProjects/Garage/scripts/garage_sac.py')