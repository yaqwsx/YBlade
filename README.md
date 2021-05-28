# YBlade - Import QBlade blades into Fusion 360

Simple script for Fusion 360 that takes [QBlade] blade description and
constructs the blade:

![result.png](result.png)

## Usage

First, use QBlade to design your blade. Then export the blade table and profile
data. Install this script and then run it. It will ask to enter the blade table
and profile file.

See example input files in [bladeExample](bladeExample).

Two bodies are generated under the root component:

- the first body is the outside shell of the blade. It is created by a sweep
  operation, so the body is completely smooth.
- then there is the infill body (you can subtract from the shell). This body is
  created by loft operations and to reduce computational complexity, it is
  simplified.

## Known limitations

Currently, only blades with a single profile are supported.