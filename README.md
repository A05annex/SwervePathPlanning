# Swerve Path Planning: 6831 A05annex

![alt text](./resources/swerve-path-planner.jpg "Swerve Path Planner")
This project is a visual 2D editor for path planning for a swerve drive FRC robot. Read a robot description
and field description into this planner as a context for path planning, then draw, tune, and save a path
that can be used as a path input in the autonomous program.

## Path Spline

The path planner uses an implementation of the
[Kochanek-Bartels Spline](https://en.wikipedia.org/wiki/Kochanek%E2%80%93Bartels_spline) modified
for interactive editing of the tangent vector to implicitly control bias and tension. There is no
continuity control.

When control points are created the tangent (derivatives) at that control point and surrounding
control points are computed using the [Cardinal-Spline](https://en.wikipedia.org/wiki/Cubic_Hermite_spline)
formulation with the default tension specified by a program constant. The tangent is adjusted using a
control handle which intuitively manipulates the shape of the spline at the control point to implicitly
edit tension and bias.

## Robot Description

The robot is described in a <tt>.json</tt> file read into the planner and displayed as the robot during
the path planning, as well as providing the drive geometry and max speed for the modules of the swerve
drive.

### Robot Description Format

The robot description is divided into 3 sections:
- **<tt>"drive"</tt>**: describes the geometry of the drive
  - **<tt>"length"</tt>**: The length of the drive (pivot axis to pivot axis) in meters.
  - **<tt>"width"</tt>**: The width of the drive (pivot axis to pivot axis) in meters.
  - **<tt>"maxSpeed"</tt>**: Tha maximum module speed (meters/sec)
- **<tt>"chassis"</tt>**: describes the geometry of the chassis (it is currently assumed the drive
  and chassis share the same centroid)
  - **<tt>"length"</tt>**: The length of the drive (pivot axis to pivot axis) in meters.
  - **<tt>"width"</tt>**: The width of the drive (pivot axis to pivot axis) in meters.
- **<tt>"bumber"</tt>**:
  - **<tt>"length"</tt>**: The length of the drive (pivot axis to pivot axis) in meters.
  - **<tt>"width"</tt>**: The width of the drive (pivot axis to pivot axis) in meters.

### Example Robot Description file

```yaml
{
  "drive": {
    "length": 0.574,
    "width": 0.577,
    "maxSpeed": 3.1951
  },
  "chassis": {
    "length": 0.762,
    "width": 0.762
  },
  "bumpers": {
    "length": 0.9144,
    "width": 0.9144
  },
}
```

## Field Description

The field is described in a <tt>.json</tt> file read into the planner and displayed as the context
for planning move paths. The field description file has 2 main elements:
- **<tt>"components"</tt>**: The field components (elements or assembles) that are generally specific
  to the competition and appear multiple times on the field.
  - **<tt>"component"</tt>**:
    - **<tt>"name"</tt>**: The name of the component. This name will be used to specify components to be
    added to the field.
    - geometry
- **<tt>"field"</tt>**: the description of the field, usually the field outline with minimal additional
  detail. Components can be specified for the field, generally with a position transformation
  and alliance color.
  - **<tt>"name"</tt>**: The name of the field, generally the name of this year's competition.
  - geometry - any of the generic geometry specifications described below.
  - **<tt>"addComponent"</tt>**:
    - **<tt>"alliance"</tt>**: The alliance color, <tt>"red"</tt>, <tt>"blue"</tt>, or <tt>"none"</tt> if the
      component is a neutral component not associated with either alliance.
    - **<tt>"componentName"</tt>**: The name of the component
    - **<tt>"transform"</tt>**: The positioning transform to be applied to the component.


