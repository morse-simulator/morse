Creating Arc objects:

The current way to configure the number of rays used by the SICK is to define a new object in the scene, a circle or semicircle with as many vertices as laser rays.
The easiest way to create the object is to Add->Mesh->Circle. Give the numer of vertices as necessary. Make sure to select the Fill option, so that there will be a central vertex and the circle will have the faces already defined.

The new object must have the following characteristics:

- Name: Its name must begin with 'Arc_', for the SICK Module to recognize it. The currently used method is to name the arcs according to the number of rays they have, for example: Arc_180, Arc_16, Arc_360

- Vertices: It is necessary that the vertex at the center of the circle is at local coordinates 0, 0, 0. This is the default case, so it should not be modified.

- Normals: For the circle to be visible in the GE, the normals of the faces must be facing up. Otherwise the object will not be displayed
