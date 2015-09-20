# april_toolkit-ros

Communicates with RethinkRobotics' Baxter.

Expects right camera to be in use before it will pick up on the port number.

Command:

java april.camera.calibrator.AprilCal -u "tcp-server://(PORT NUMBER)" -c CALIBRATION_METHOD -p kclength=4 -m 0.0381

Examples to replace (PORT NUMBER):
12345

Example inputs to replace CALIBRATION METHOD:
```
april.camera.models.CaltechInitializer
april.camera.models.AngularPolynomialInitializer
april.camera.models.RadialPolynomialInitializer
```
