# Raytracer-AABB-Tree
 
This project implements a raytracer in C++ from scratch, made without the use of any graphic libraries.

Objects (in .off format) are loaded into the scene within `main.cpp` and an output .png image is generated.

The raytracer is implemented using a perspective view and supports shadows, and reflections. The main focus of the program is to use an AABB Tree which spacially partitions triangle faces into a binary search tree in order to optimise ray intersection computations by quickly eliminating large subsets of triangles from the list of triangles that need to be checked for a possible intersection with the ray.

By implementing an AABB Tree, raytracer performance is massively increased compared to the brute force method.

Wereas the provided `dragon.off` model with nearly 1 million triangles would typically take hours to render using the brute force method raytracer, the AABB Tree version renders the model in about 15 seconds.

For comparison, the `bunny.off` model with about 1000 triangles takes 14 times as long to render using brute force than the 1 million triangle `dragon.off` with AABB Tree:

### Runtimes (with shadows & reflections)
| Model  | Time |
| ------------- | ------------- |
| bunny (brute force)  | 3:30 mins  |
| dragon (AABB Tree)  | 15-20 secs  |


## Sample Images:
Below are some sample images rendered using the AABB Tree raytracer.

Note that the scene is rendered with 7 light sources for shadows and that reflection computation has a ray bounce recursion depth of 5. Final images are rendered in 640x480 resolution.

![](my_images/raytrace_dragon_tree.png?raw=true)

Fig 1: The `dragon.off` model rendered with shadows and reflections.

![](my_images/raytrace_dragon_tree_without_reflections.png?raw=true)

Fig 2: The `dragon.off` model rendered shadows but no reflections.

![](my_images/raytrace_bunny_tree.png?raw=true)

Fig 3: The `bunny.off` model rendered with shadows and reflections.
