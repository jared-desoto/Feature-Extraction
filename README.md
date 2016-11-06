# Feature-Extraction

The basic algorithm for detecting feature lines of a mesh can be broken down into a few basic parts: 
1. Calculation of intermediary values 
2. Processing of regular triangles 
3. Processing of singular triangles 


1. There are several intermediary values that must be calculated before any processing can be done: 
• Dihedral angle 𝜃: the smaller angle between any two faces in the mesh. This is an edge-defined value. 
• Mean curvature 𝐻=|𝑒|cos(𝜃/2), where 𝑒 is the magnitude of the edge on which 𝐻 is defined. 
• Shape operators 𝑆(𝑒)=𝐻(e× 𝑁e)(𝑒×𝑁e) and      
                  𝑆(𝑝)=1/2 * Σ(e∋p)<𝑁p,Ne>S(e),where 𝑆𝑒 is the edge-based shape operator, 
    𝑆(𝑝) is the vertex-based shape operator, 𝑒 is a unit vector in the direction of an edge (sign does not matter), 
    𝑁! is a unit vector in the direction of the average of the normal vectors of the faces adjacent to an edge, 
    and 𝑁! is the normal vector at a point, calculated by a weighted average of the normal vectors of the adjacent faces. 
• Principal curvatures 𝜅max and 𝜅min, which are the eigenvalues of the vertex-based shape operators with the highest absolute value. 
    We also calculate the corresponding eigenvectors 𝑘max and 𝑘min. These are all vertex-based quantities. 
    The signs of these values are chosen such that the dot products of any two 𝜅! on a triangle will be positive. 
    If such signs can be chosen, the triangle is said to be regular. Otherwise, the triangle is said to be singular. 
• Extremalities 𝑒max and 𝑒min, which are essentially a measure of local flatness around the vertex on which they are defined. 

    e(p) = 1/area(star(p))Σ(T∋p)area(T)<∇Ki(T), Ki(p)>
    Here, area(star(p)) refers to the summation of the areas of the triangles adjacent to the vertex p.
    This value is only processed on regular triangles

2. Regular triangles are detected by simply testing all possible combinations of signs for the principal curvatures on the triangle, 
and checking to see that for some combination of these signs, all of the principal curvature vectors exist in the same octant. 
Once the regular triangles are detected, extremalities can be calculated using the principal curvature signs that were previously chosen. 
From here, the vertices corresponding to positive feature lines can be found using the following three conditions: 
      i.  <∇𝑒max, Σ(p∋T)𝜅max(𝑝)> < 0 
      ii. |Σ(p∋T)𝜅max(𝑝i)| < |Σ(p∋T)𝜅min(𝑝i)|
      iii. at least one 𝜅max sufficiently close to 0 
We proceed analogously for negative feature lines, simply switching equality signs in the first two conditions. 
To determine whether a principal curvature value is sufficiently close to 0, we calculate a threshold value,
which will be discussed later. 
 
3. Singular triangles do not allow for consistent calculation of extremalities, so they reflect the values of the regular triangles 
    that surround them, provided that there exists more than 1 adjacent regular triangle. 
    If a singular triangle borders either two or three regular triangles that are also on feature lines, 
    it is considered to be on a feature line. Otherwise, it is not considered a feature line face. 
 
Additional Steps 
 
1. We have set up a threshold that is set automatically to determine whether or not a principal curvature value is sufficiently close 
    to 0 for condition iii in feature line identification. This threshold is set based on the assumption that feature lines will 
    coincide with areas of significantly high curvature in relation to the areas around them.  
 

Conclusion 
 
Feature line detection is important in identifying the salient regions of surface meshes. 
A key to easily and accurately detecting feature lines on different meshes is the real-time tweaking of certain threshold parameters 
(included in the GUI). Reducing groups of feature line faces to single lines allows for the creation of a neat set of feature lines 
that accurately represents the most prominent features of the mesh. A possible application of this project is a mesh reduction that 
will preserve the most prominent regions of the mesh, thereby preserving the overall appearance of the mesh. 
