# BFH HuCE-roboticsLab Python Framework

This project contains the BFH HuCE-roboticsLab Python Framework used for R&D projects and lectures.

### Table of Contents

1. [Software Requirements](#requirements)
1. [Initialization](#initialization)

## <a name="requirements"></a>Software Requirements

To install requirements run

```bash
python3 -m pip install -r requirements.txt
```

## <a name="initialization"></a>Initialization

The framework depends on the git submodule `Protocol`. After cloning this git repository you need to initialize the submodules with:

```bash
git submodule update --init
```

This will clone the `Protocol` submodule.
After that you must generate the Python grpc and protobuf sources with:

```
./update-grpc.sh
```

**You will need to run this script again if the `Protocol` is updated.**
