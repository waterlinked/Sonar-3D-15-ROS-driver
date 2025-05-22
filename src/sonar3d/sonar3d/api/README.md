# Sonar-3D-15-api-example

API example for reading and decoding the data from the Water Linked Sonar 3D-15 with Python

# Setup

In your python environment you need to install the requirements.txt 
```consol
pip install -r requirements.txt
```
!!! Note
    It is important to use the 4.24.2 protobuf version in the requirements file. If you already have protbuf installed you need to downgrade or look at alternative 2.

# Using

Collect the data from a sonar, press ctrl-c to stop collecting data:

```consol
python save_sonar_data.py --file myfile.sonar
```

Inspecting the collected data:

```consol
python inspect_sonar_data.py --file myfile.sonar
```

Optionally you can add the a `--save` argument to save the data to file:
- RangeImage to 3d voxels in the `.xzy` format (supported by for example MeshLab) in the Sonars frame of reference
- BitmapImage to grayscale image in the `.pgm` format (supported by Photoshop, GIMP etc)

The save function is provided as an example of how to process the data from the Sonar, please modify as you see fit.
_____
Interface the sonar through the API (v1.4.0 or newer):

```consol
python interface_sonar_api.py --ip <SONAR IP>
```
Prints information about the Sonar (version, temperature...). Optionally you can control the soar with the arguments:
- `--speed`: Change the speed of sound used by the sonar (takes ~20s to complete)
- `--acoustics`: Enable or disable the acoustics on the sonar (choices [enable, disable]) 

# Advanced

If you are already using another version of protobuf this is no problem, as you can take the 'sonar-3d-15-protocol.proto' file and generate a new 'sonar_3d_15_protocol_pb2.py with the correct version. For this you need to install the protobuf compiler to match your protobuf pip installed version (runner). Please see this doc: https://protobuf.dev/support/cross-version-runtime-guarantee/

## Example of compiling a python file from the proto file
```consol
protoc --python_out=. sonar-3d-15-protocol.proto  
```