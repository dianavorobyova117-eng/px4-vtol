#WARNING
You should first configure your ssh keys for git and github!
It might be helpful to add proxy to git even if you have system proxy.

# Usage
```shell
git clone git@github.com:SYSU-HILAB/px4-v1.14.0-stable.git
bash Tool/setup/ubuntu.sh
git submodule update --init --recursive
```
## exp. fmu-6c
```shell
make px4_fmu-v6c upload
```
