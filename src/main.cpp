/* RAYTRACER AABB-TREE
 *
 * Version: V1-0 (Mar 24, 2022)
 *
 * Run Times (with shadows & reflections):
 * - Bunny (brute force): 3:30min
 * - Dragon (AABBTree): 15-20sec
 * 
 */

////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <fstream>
#include <algorithm>
#include <numeric>

#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;


////////////////////////////////////////////////////////////////////////////////
// Class to store tree
////////////////////////////////////////////////////////////////////////////////
class AABBTree
{
public:
    class Node
    {
    public:
        AlignedBox3d bbox;
        int parent;   // Index of the parent node (-1 for root)
        int left;     // Index of the left child (-1 for a leaf)
        int right;    // Index of the right child (-1 for a leaf)
        int triangle; // Index of the node triangle (-1 for internal nodes)
    };

    std::vector<Node> nodes;
    int root;

    AABBTree() = default;                           // Default empty constructor
    AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh
};


// FUNCTION DECLARATIONS: //
std::tuple<bool, Vector3d> intersects_triangle(Vector3d, Vector3d, Vector3d, Vector3d, Vector3d);
std::tuple<bool, double> intersects_sphere(Vector3d, double, Vector3d, Vector3d);
std::tuple<bool, Vector3d> intersects_pgram(Vector3d, Vector3d, Vector3d, Vector3d, Vector3d);
int recursive_build(int, int, int, AABBTree&, std::vector<int>&);
bool sort_x(int, int);
bool sort_y(int, int);
bool sort_z(int, int);


////////////////////////////////////////////////////////////////////////////////
// Scene setup, global variables
////////////////////////////////////////////////////////////////////////////////
const std::string data_dir = DATA_DIR;
const std::string filename("raytrace.png");
const std::string mesh_filename(data_dir + "dragon.off"); // NOTE: Change object file to render here

//Camera settings
const double focal_length = 2;
const double field_of_view = 0.7854; //45 degrees
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 2);

//Maximum number of recursive calls
const int max_bounce = 5;

// Triangle Mesh
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)
AABBTree bvh;

// MY GLOBAL VARIABLES: //
MatrixXd centroids;

// Objects
std::vector<Vector3d> sphere_centers;
std::vector<double> sphere_radii;
std::vector<Matrix3d> parallelograms;

//Material for the object, same material for all objects
const Vector4d obj_ambient_color(0.0, 0.5, 0.0, 0);
const Vector4d obj_diffuse_color(0.5, 0.5, 0.5, 0);
const Vector4d obj_specular_color(0.2, 0.2, 0.2, 0);
const double obj_specular_exponent = 256.0;
const Vector4d obj_reflection_color(0.7, 0.7, 0.7, 0);

// Precomputed (or otherwise) gradient vectors at each grid node
const int grid_size = 20;
std::vector<std::vector<Vector2d>> grid;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector4d> light_colors;
//Ambient light
const Vector4d ambient_light(0.2, 0.2, 0.2, 0);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    vertices.resize(nv, 3);
    facets.resize(nf, 3);
    for (int i = 0; i < nv; ++i)
    {
        in >> vertices(i, 0) >> vertices(i, 1) >> vertices(i, 2);
    }
    for (int i = 0; i < nf; ++i)
    {
        int s;
        in >> s >> facets(i, 0) >> facets(i, 1) >> facets(i, 2);
        assert(s == 3);
    }

    //Spheres

    // My spheres
    sphere_centers.emplace_back(-1.8, -0.6, -0.3);
    sphere_radii.emplace_back(1);

    //sphere_centers.emplace_back(-1.5, -0.3, -0.7);
    //sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(2.5, 0, -3);
    sphere_radii.emplace_back(1);

    // Additional Spheres nicely placed for good reflections
    /*
    sphere_centers.emplace_back(10, 0, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(7, 0.05, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(4, 0.1, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(1, 0.2, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-2, 0.4, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-5, 0.8, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-8, 1.6, 1);
    sphere_radii.emplace_back(1);
    */

    //parallelograms
    parallelograms.emplace_back();
    parallelograms.back() << -100, 100, -100,
        -1.25, 0, -1.2,  
        -100, -100, 100;

    
    //setup tree
    bvh = AABBTree(vertices, facets);


    //Lights

    // added front facing test light
    //light_positions.emplace_back(0, 4, 0);
    //light_colors.emplace_back(16, 16, 16, 0);
    
    // main lights

    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Code
////////////////////////////////////////////////////////////////////////////////

AlignedBox3d bbox_from_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c)
{
    AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
}

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F)
{
    // Compute the centroids of all the triangles in the input mesh
    centroids.resize(F.rows(), V.cols()); // changed to making centroids global so that sort_x,y,z can use them. Instead of initializing here, we resize the undefined global declaration.
    centroids.setZero();
    for (int i = 0; i < F.rows(); ++i)
    {
        for (int k = 0; k < F.cols(); ++k)
        {
            centroids.row(i) += V.row(F(i, k));
        }
        centroids.row(i) /= F.cols();
    }

    //std::cout << "Initial centroids.rows() = " << centroids.rows() << std::endl;

    // create an "array" (std::vector) that stores the indexes of the centroids.
    // This array will then be sorted based on the centroids read from that index position
    std::vector<int> c_indexes;

    // initialize to a list of increasing order indices. It will later be sorted based on the value of the centroids at those indices.
    for (int i = 0; i < centroids.rows(); ++i) {
        c_indexes.push_back(i);
    }

    // setup a leaf node for each triangle
    for (int i = 0; i < facets.rows(); ++i) {
        this->nodes.push_back(Node()); // create a new node for a triangle
        this->nodes[i].triangle = i; //set triangle to be an index of a facet (here we just conviniently set them up in ascending order with i)
        this->nodes[i].parent = -1; //set parent to -1 to indicate it has no parent yet
        this->nodes[i].left = -1;
        this->nodes[i].right = -1;
        this->nodes[i].bbox = bbox_from_triangle(vertices.row(facets(i, 0)), vertices.row(facets(i, 1)), vertices.row(facets(i, 2)));
    }

    // Split each set of primitives into 2 sets of roughly equal size,
    // based on sorting the centroids along one direction or another.


    // recursive portion starts below

    // NOTE that we don't save any return value because recursive_build() returns an int
    // that is used in its own internal recursive implementation,
    // however that is ok because we passed a reference to the tree,
    // meaning that now the tree has been modified and is built following this line of code.

    this->root = recursive_build(0, c_indexes.size() - 1, -1, *this, c_indexes); // we set cur_root to -1 in the initial call so that the master root has parent -1 as expected. function return root index.

}

int recursive_build(int start_index, int end_index, int cur_root, AABBTree &t, std::vector<int> &c_indexes) {
    
    // iterate over c_indexes of size:
    int c_size = end_index - start_index + 1;
    
    // sort by centroids

    //std::cout << "centroids.rows() = " << centroids.rows() << std::endl;

    // create a bounding box that contains all triangle boxes in the given range
    AlignedBox3d bbox_box;
    for (int i = start_index; i < end_index + 1; ++i) {
        bbox_box.extend(t.nodes[c_indexes[i]].bbox);
    }

    // determine the max axis

    Vector3d d = bbox_box.diagonal();

    double max_axis = std::max(std::max(d[0], d[1]), d[2]);

    //std::cout << "Before sort - c_indexes.size() = " << c_indexes.size() << std::endl;
    //std::cout << "In recursive_build: start_index = " << start_index << ", end_index = " << end_index << std::endl;

    if (max_axis == d[0]) {
        // max_axis is the x axis
        std::sort(c_indexes.begin() + start_index, c_indexes.begin() + end_index + 1, sort_x);
    }
    else if (max_axis == d[1]) {
        // max_axis is the y axis
        std::sort(c_indexes.begin() + start_index, c_indexes.begin() + end_index + 1, sort_y);
    }
    else {
        // max_axis is the z axis
        std::sort(c_indexes.begin() + start_index, c_indexes.begin() + end_index + 1, sort_z);
    }

    //std::cout << "After sort - c_indexes.size() = " << c_indexes.size() << std::endl;


    // create a new current parent node using bbox_box
    
    //AABBTree::Node nnode = AABBTree::Node::Node(); // alternate way instead of reference by index

    t.nodes.push_back(AABBTree::Node::Node());
    int nnode_index = t.nodes.size() - 1;

    int left_recursive = -1;
    int right_recursive = -1;
    if (c_size > 2) { // only recurse if the list of elements to sort is 2 or greater
        left_recursive = recursive_build(start_index, ((end_index - start_index)/ 2) + start_index, nnode_index, t, c_indexes); // recursive call S1

        if (c_size > 3) { // only do right recurive call if we can still split in at least two groups of two, else right branch equals the 3rd (last) element
            right_recursive = recursive_build(((end_index - start_index) / 2) + start_index + 1, end_index, nnode_index, t, c_indexes); // recursive call S2
        }
        else {
            right_recursive = c_indexes[end_index];
        }
    }
    else { // base case. two triangles left become left and right branch of internal node
        left_recursive = c_indexes[start_index];
        right_recursive = c_indexes[end_index];
    }

    // set attributes for new internal node
    t.nodes[nnode_index].bbox = bbox_box;
    t.nodes[nnode_index].parent = cur_root;
    t.nodes[nnode_index].triangle = -1; // internal node
    t.nodes[nnode_index].left = left_recursive;
    t.nodes[nnode_index].right = right_recursive;

    return nnode_index; //i.e. the index of the newly added root
}

// custom sort functions. Takes the index of two centroids and
// returns true if the first element is smaller than the second
bool sort_x(int a, int b) {
    //std::cout << "centroids.rows() = " << centroids.rows() << ", a = " << a << ", b = " << b << ", centroids.row(a)[0] = " << (centroids.row(a))[0] << " after." << std::endl;
    return centroids.row(a)[0] < centroids.row(b)[0];
}

bool sort_y(int a, int b) {
    return centroids.row(a)[1] < centroids.row(b)[1];
}

bool sort_z(int a, int b) {
    return centroids.row(a)[2] < centroids.row(b)[2];
}


////////////////////////////////////////////////////////////////////////////////
// Intersection code
////////////////////////////////////////////////////////////////////////////////

double ray_triangle_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &a, const Vector3d &b, const Vector3d &c, Vector3d &p, Vector3d &N)
{
    // Compute whether the ray intersects the given triangle.
    // If you have done the parallelogram case, this should be very similar to it.

    // implement the intersection between the ray and the triangle given its three vertices.
    //return t or -1 if no intersection

    double t = -1;

    // points
    const Vector3d A = a;
    const Vector3d B = b;
    const Vector3d C = c;

    // vector (2 sides of triangle required to compute normal)
    const Vector3d t_u = B - A;
    const Vector3d t_v = C - A;

    // Check if the ray intersects with the triangle

    std::tuple<bool, Vector3d> result;
    result = intersects_triangle(A, B, C, ray_origin, ray_direction);

    if ((std::get<0>(result)) == false) //if ray intersects with triangle
    {
        return -1;
    }
    else {
        t = std::get<1>(result)(2);

        //set the correct intersection point, update p and N to the correct values

        p = ray_origin + (t * ray_direction); //returns a vector with p(t) = origin + (t * direction)
        N = t_u.cross(t_v).normalized();
        return t;
    }
}


// Helper funtion for ray_triangle_intersection adapted from intersects_pgram().
// 
// Given the three vertices of a triangle along with a ray,
// the function returns a tuple indicating whether the ray intersects with the triangle
// and if so, the coordinates of the intersection.
std::tuple<bool, Vector3d> intersects_triangle(Vector3d a, Vector3d b, Vector3d c, Vector3d ray_ori, Vector3d ray_dir)
{
    // some math for reference:
    /// ray: p(t) = e + t*d -> p(t) = origin + t*direction
    //Vector3d ray = ray_ori + t * ray_dir; /// t is essentially how long the vector has to be to intersect the triangle

    /// save triangle vertices (think of these as points, not vectors)

    // t1 triangle
    Vector3d t1_a = a;
    Vector3d t1_b = b;
    Vector3d t1_c = c;

    /// compute the intersection of ray and triangle (return true if it intersects ray)

    /// CHECK INTERSECTION FOR TRIANGLE 1:

    /// create a matrix that combines three equations to solve for three unknowns
    /// this section essentially solves for u,v,t in p(t) = ray_ori + t * ray_dir = f(u,v) = a + u*(b-a) + v(c-a)
    /// See textbook p.88-89

    Matrix3d A1;
    A1 << t1_a(0) - t1_b(0), t1_a(0) - t1_c(0), ray_dir(0), /// solve for x axis
        t1_a(1) - t1_b(1), t1_a(1) - t1_c(1), ray_dir(1), /// solve for y axis
        t1_a(2) - t1_b(2), t1_a(2) - t1_c(2), ray_dir(2); /// solve for z axis

    Matrix3d m1_u;
    m1_u << t1_a(0) - ray_ori(0), t1_a(0) - t1_c(0), ray_dir(0), /// solve for x axis
        t1_a(1) - ray_ori(1), t1_a(1) - t1_c(1), ray_dir(1), /// solve for y axis
        t1_a(2) - ray_ori(2), t1_a(2) - t1_c(2), ray_dir(2); /// solve for z axis

    Matrix3d m1_v;
    m1_v << t1_a(0) - t1_b(0), t1_a(0) - ray_ori(0), ray_dir(0), /// solve for x axis
        t1_a(1) - t1_b(1), t1_a(1) - ray_ori(1), ray_dir(1), /// solve for y axis
        t1_a(2) - t1_b(2), t1_a(2) - ray_ori(2), ray_dir(2); /// solve for z axis

    Matrix3d m1_t;
    m1_t << t1_a(0) - t1_b(0), t1_a(0) - t1_c(0), t1_a(0) - ray_ori(0), /// solve for x axis
        t1_a(1) - t1_b(1), t1_a(1) - t1_c(1), t1_a(1) - ray_ori(1), /// solve for y axis
        t1_a(2) - t1_b(2), t1_a(2) - t1_c(2), t1_a(2) - ray_ori(2); /// solve for z axis

    double res1_u = m1_u.determinant() / A1.determinant();
    double res1_v = m1_v.determinant() / A1.determinant();
    double res1_t = m1_t.determinant() / A1.determinant();


    /// Check intersection results for triangle and determine if this constitutes an intersection with ray.
    if (res1_t > 0.0 && res1_u >= 0.0 && res1_v >= 0.0 && (res1_u + res1_v) <= 1.0) {
        const Vector3d intersect_point(res1_u, res1_v, res1_t);
        //std::cout << "1 - res1_u: " << res1_u << " res1_v: " << res1_v << " res1_t: " << res1_t << std::endl;
        return std::make_tuple(true, intersect_point); // return intersection
    }

    // If triangle doesn't intersect the ray
    
    //std::cout << "Dummy return. The following did not intersect - res1_u: " << res1_u << " res1_v: " << res1_v << " res1_t: " << res1_t << std::endl;
    const Vector3d dummy_return(-1, -1, -1);
    return std::make_tuple(false, dummy_return);
}

//Compute the intersection between a ray and a paralleogram, return -1 if no intersection
double ray_parallelogram_intersection(const Vector3d& ray_origin, const Vector3d& ray_direction, int index, Vector3d& p, Vector3d& N)
{
    double t = -1;

    // Added in updated version of assignment for easy access to pgram coordinates from the vector "array" that stored all coordinates
    const Vector3d pgram_origin = parallelograms[index].col(0);
    const Vector3d A = parallelograms[index].col(1);
    const Vector3d B = parallelograms[index].col(2);
    const Vector3d pgram_u = A - pgram_origin;
    const Vector3d pgram_v = B - pgram_origin;

    // Check if the ray intersects with the parallelogram

    std::tuple<bool, Vector3d> result;
    result = intersects_pgram(pgram_origin, pgram_u, pgram_v, ray_origin, ray_direction);

    if ((std::get<0>(result)) == false) //if ray intersects with pgram
    {
        return -1;
    }
    else {
        t = std::get<1>(result)(2);

        p = ray_origin + (t * ray_direction); //returns a vector with p(t) = origin + (t * direction)
        N = pgram_v.cross(pgram_u).normalized();
        return t;
    }
}


// Given the origin and two vectors that define a parallelogram along with a ray,
// the function returns a tuple indicating whether the ray intersects with the polygon
// and if so, the coordinates of the intersection.
std::tuple<bool, Vector3d> intersects_pgram(Vector3d origin, Vector3d u, Vector3d v, Vector3d ray_ori, Vector3d ray_dir)
{
    /// split the pgram into two triangles

    // some math for reference:
    /// ray: p(t) = e + t*d -> p(t) = origin + t*direction
    //Vector3d ray = ray_ori + t * ray_dir; /// t is essentially how long the vector has to be to intersect the triangle


    /// save triangle vertices (think of these as points, not vectors)

    // t1 triangle
    Vector3d t1_a = origin;
    Vector3d t1_b = origin + u;
    Vector3d t1_c = origin + v;

    // t2 triangle
    Vector3d t2_a = t1_b;
    Vector3d t2_b = t1_b + v;
    Vector3d t2_c = t1_c;

    /// compute the intersection of ray and each triangle (return true if true for either)

    /// CHECK INTERSECTION FOR TRIANGLE 1:

    /// create a matrix that combines three equations to solve for three unknowns
    /// this section essentially solves for u,v,t in p(t) = ray_ori + t * ray_dir = f(u,v) = a + u*(b-a) + v(c-a)
    /// See textbook p.88-89

    Matrix3d A1;
    A1 << t1_a(0) - t1_b(0), t1_a(0) - t1_c(0), ray_dir(0), /// solve for x axis
        t1_a(1) - t1_b(1), t1_a(1) - t1_c(1), ray_dir(1), /// solve for y axis
        t1_a(2) - t1_b(2), t1_a(2) - t1_c(2), ray_dir(2); /// solve for z axis

    Matrix3d m1_u;
    m1_u << t1_a(0) - ray_ori(0), t1_a(0) - t1_c(0), ray_dir(0), /// solve for x axis
        t1_a(1) - ray_ori(1), t1_a(1) - t1_c(1), ray_dir(1), /// solve for y axis
        t1_a(2) - ray_ori(2), t1_a(2) - t1_c(2), ray_dir(2); /// solve for z axis

    Matrix3d m1_v;
    m1_v << t1_a(0) - t1_b(0), t1_a(0) - ray_ori(0), ray_dir(0), /// solve for x axis
        t1_a(1) - t1_b(1), t1_a(1) - ray_ori(1), ray_dir(1), /// solve for y axis
        t1_a(2) - t1_b(2), t1_a(2) - ray_ori(2), ray_dir(2); /// solve for z axis

    Matrix3d m1_t;
    m1_t << t1_a(0) - t1_b(0), t1_a(0) - t1_c(0), t1_a(0) - ray_ori(0), /// solve for x axis
        t1_a(1) - t1_b(1), t1_a(1) - t1_c(1), t1_a(1) - ray_ori(1), /// solve for y axis
        t1_a(2) - t1_b(2), t1_a(2) - t1_c(2), t1_a(2) - ray_ori(2); /// solve for z axis

    double res1_u = m1_u.determinant() / A1.determinant();
    double res1_v = m1_v.determinant() / A1.determinant();
    double res1_t = m1_t.determinant() / A1.determinant();


    /// CHECK INTERSECTION FOR TRIANGLE 2:

    /// create a matrix that combines three equations to solve for three unknowns
    /// this section essentially solves for u,v,t in p(t) = ray_ori + t * ray_dir = f(u,v) = a + u*(b-a) + v(c-a)
    /// See textbook p.88-89

    Matrix3d A2;
    A2 << t2_a(0) - t2_b(0), t2_a(0) - t2_c(0), ray_dir(0), /// solve for x axis
        t2_a(1) - t2_b(1), t2_a(1) - t2_c(1), ray_dir(1), /// solve for y axis
        t2_a(2) - t2_b(2), t2_a(2) - t2_c(2), ray_dir(2); /// solve for z axis

    Matrix3d m2_u;
    m2_u << t2_a(0) - ray_ori(0), t2_a(0) - t2_c(0), ray_dir(0), /// solve for x axis
        t2_a(1) - ray_ori(1), t2_a(1) - t2_c(1), ray_dir(1), /// solve for y axis
        t2_a(2) - ray_ori(2), t2_a(2) - t2_c(2), ray_dir(2); /// solve for z axis

    Matrix3d m2_v;
    m2_v << t2_a(0) - t2_b(0), t2_a(0) - ray_ori(0), ray_dir(0), /// solve for x axis
        t2_a(1) - t2_b(1), t2_a(1) - ray_ori(1), ray_dir(1), /// solve for y axis
        t2_a(2) - t2_b(2), t2_a(2) - ray_ori(2), ray_dir(2); /// solve for z axis

    Matrix3d m2_t;
    m2_t << t2_a(0) - t2_b(0), t2_a(0) - t2_c(0), t2_a(0) - ray_ori(0), /// solve for x axis
        t2_a(1) - t2_b(1), t2_a(1) - t2_c(1), t2_a(1) - ray_ori(1), /// solve for y axis
        t2_a(2) - t2_b(2), t2_a(2) - t2_c(2), t2_a(2) - ray_ori(2); /// solve for z axis

    double res2_u = m2_u.determinant() / A2.determinant();
    double res2_v = m2_v.determinant() / A2.determinant();
    double res2_t = m2_t.determinant() / A2.determinant();


    /// Check intersection results for each triangle and determine if this constitutes an intersection with the pgram.
    /// If either triangle intersects, than the pgram intersects.

    bool t1_intersects = false;
    bool t2_intersects = false;

    if (res1_t > 0.0 && res1_u >= 0.0 && res1_v >= 0.0 && (res1_u + res1_v) <= 1.0) {
        t1_intersects = true; //ray intersects with triangle 1
    }

    if (res2_t > 0.0 && res2_u >= 0.0 && res2_v >= 0.0 && (res2_u + res2_v) <= 1.0) {
        t2_intersects = true; //ray intersects with triangle 2
    }


    // If both triangles intersect the ray
    // (this shouldn't usually happen but is possible in theory on the diagonal of the pgram
    // at the boundary of where the two triangles meet. The choice would likely be arbitrary anyways
    // as chances are both t values would be the same.)
    if (t1_intersects && t2_intersects) {

        //printf("Both t1 & t2 intersect for triangle. This shouldn't typically happen.\n");

        // return t1
        if (res1_t <= res2_t) {
            const Vector3d intersect_point(res1_u, res1_v, res1_t);
            //std::cout << "1 - res1_u: " << res1_u << " res1_v: " << res1_v << " res1_t: " << res1_t << std::endl;
            return std::make_tuple(true, intersect_point);
        }
        // return t2
        else {
            const Vector3d intersect_point(res2_u, res2_v, res2_t);
            //std::cout << "2 - res2_u: " << res2_u << " res2_v: " << res2_v << " res2_t: " << res2_t << std::endl;
            return std::make_tuple(true, intersect_point);
        }
    }


    // If only one triangle intersects the ray, check which one and return point accordingly
    if (t1_intersects) {
        const Vector3d intersect_point(res1_u, res1_v, res1_t);
        //std::cout << "1 - res1_u: " << res1_u << " res1_v: " << res1_v << " res1_t: " << res1_t << std::endl;
        return std::make_tuple(true, intersect_point);
    }

    if (t2_intersects) {
        const Vector3d intersect_point(res2_u, res2_v, res2_t);
        //std::cout << "2 - res2_u: " << res2_u << " res2_v: " << res2_v << " res2_t: " << res2_t << std::endl;
        return std::make_tuple(true, intersect_point);
    }

    // If no triangle intersects the ray (i.e. ray did not hit parallelogram)

    //std::cout << "Dummy return. The following did not intersect - res1_u: " << res1_u << " res1_v: " << res1_v << " res1_t: " << res1_t << std::endl;
    const Vector3d dummy_return(-1, -1, -1);
    return std::make_tuple(false, dummy_return);
}

// function taken from assignment 3
//Compute the intersection between a ray and a sphere, return -1 if no intersection
double ray_sphere_intersection(const Vector3d& ray_origin, const Vector3d& ray_direction, int index, Vector3d& p, Vector3d& N)
{
    double t = -1;

    std::tuple<bool, double> result; // tuple of format: (true if interection, t value)

    Vector3d sphere_center = sphere_centers[index];
    double sphere_radius = sphere_radii[index];

    // print for debug info
    //std::cout << "sphere_centers[" << index << "] = " << sphere_centers[index] << std::endl; //sphere_centers[index] accesses correct data

    result = intersects_sphere(sphere_center, sphere_radius, ray_origin, ray_direction);


    if ((std::get<0>(result)) == false) //if ray intersects with sphere
    {
        return -1;
    }
    else
    {

        t = std::get<1>(result);

        p = ray_origin + (t * ray_direction);
        N = (p - sphere_center).normalized();

        return t;
    }

    return -1;
}

// Given a ray and a sphere center and radius, returns whether the ray intersects the sphere
// and if so, the t value at the intersection.
std::tuple<bool, double> intersects_sphere(Vector3d center, double radius, Vector3d ray_ori, Vector3d ray_dir)
{
    /*
    std::cout   << "center: (" << center(0) << "," << center(1) << "," << center(2) << ") radius: " << radius
                << " ray_ori: (" << ray_ori(0) << "," << ray_ori(1) << "," << ray_ori(2)
                << ") ray_dir: (" << ray_dir(0) << "," << ray_dir(1) << "," << ray_dir(2) << ")" << std::endl;
    */

    // Check for ray and sphere intersection following equation in textbook p.86-87

    // for ray p(t) = e + t*d and sphere f(p) = (p - c) dot (p - c) - R^2 = 0

    Vector3d e_c = ray_ori - center;

    // Creating an equation of the form a*t^2 + b*t + c = 0

    double a = ray_dir.dot(ray_dir);
    double b = (2 * ray_dir).dot(e_c);
    double c = e_c.dot(e_c) - (radius * radius);

    // check the discriminant b^2 - 4*a*c -> (if d < 0: no solutions; d = 0: one solution; d > 0: two solutions )

    double discriminant = (b * b) - (4 * a * c);

    // Get some info about discriminant for debug
    /*
    if (discriminant >= 0) {
        std::cout << "discriminant: " << discriminant << std::endl;
    }
    */

    if (discriminant < 0) { //no solutions
        return std::make_tuple(false, 1.0); //double is never checked. number given is arbitrary but helps debug if they all return differently.
    }
    else if (discriminant == 0) { // one solution (sphere edge)
        // Solve the quadratic equation ( -b +- sqrt(b^2 - 4ac) ) / 2a to find t at intersection point
        double t = ((-b) + sqrt(discriminant)) / (2 * a); //both +/- solutions are the same, we compute + arbitrarily

        if (t > 0) { // make sure intersection is not behind camera (say the camera is inside the sphere)
            return std::make_tuple(true, t);
        }
    }
    else if (discriminant > 0) { // two solutions
        // Solve the quadratic equation ( -b +- sqrt(b^2 - 4ac) ) / 2a to find t at intersection point
        double t1 = ((-b) + sqrt(discriminant)) / (2 * a);
        double t2 = ((-b) - sqrt(discriminant)) / (2 * a);

        if (t1 <= t2) { //flipped equality sign from >= to <= due to type
            if (t1 > 0) { // make sure intersection is not behind camera (say the camera is inside the sphere)
                return std::make_tuple(true, t1);
            }
        }
        else {
            if (t2 > 0) { // make sure intersection is not behind camera (say the camera is inside the sphere)
                return std::make_tuple(true, t2);
            }
        }
        return std::make_tuple(false, 2.0); //double is never checked. number given is arbitrary but helps debug if they all return differently.
    }

    // The code should in theory never get here
    return std::make_tuple(false, 0.0); //double is never checked. number given is arbitrary but helps debug if they all return differently.
}





bool ray_box_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const AlignedBox3d &box)
{
    // Compute whether the ray intersects the given box.
    // we are not testing with the real surface here anyway.

    // See textbook p.310-313 for details about these computations
    
    // Find max and min values for x,y,z of bounding box

    Vector3d box_max = box.max();
    Vector3d box_min = box.min();

    double xmax = box_max(0);
    double ymax = box_max(1);
    double zmax = box_max(2);

    double xmin = box_min(0);
    double ymin = box_min(1);
    double zmin = box_min(2);

    //std::cout << box.TopRightCeil << std::endl;
    //std::cout << "box.max = " << box.max() << std::endl;

    // compute max/min t for x,y,z
    
    double tmp;

    double t_xmax = (xmax - ray_origin(0)) / ray_direction(0);
    double t_xmin = (xmin - ray_origin(0)) / ray_direction(0);

    double t_ymax = (ymax - ray_origin(1)) / ray_direction(1);
    double t_ymin = (ymin - ray_origin(1)) / ray_direction(1);

    double t_zmax = (zmax - ray_origin(2)) / ray_direction(2);
    double t_zmin = (zmin - ray_origin(2)) / ray_direction(2);


    // depending on ray direction, max and min may actually be inverted so we flip them

    if (t_xmax < t_xmin) {
        tmp = t_xmax;
        t_xmax = t_xmin;
        t_xmin = tmp;
    }
    if (t_ymax < t_ymin) {
        tmp = t_ymax;
        t_ymax = t_ymin;
        t_ymin = tmp;
    }
    if (t_zmax < t_zmin) {
        tmp = t_zmax;
        t_zmax = t_zmin;
        t_zmin = tmp;
    }
    

    // check if x,y,z ranges overlap, if so, then the ray intersects the bounding box

    if ((t_xmin > t_ymax) || (t_ymin > t_xmax)) {
        return false;
    }

    t_xmax = std::min(t_xmax, t_ymax);
    t_xmin = std::max(t_xmin, t_ymin);

    if ((t_xmin > t_zmax) || (t_zmin > t_xmax)) {
        return false;
    }

    return true;
}

// given a tree, the function returns a vector containing the facet indices for only the facets
// that have a bounding box that actually intersects with the ray.
std::vector<int> search_tree(const Vector3d& ray_origin, const Vector3d& ray_direction, AABBTree::Node& cur_root) {

    //std::cout << "search_tree() called." << std::endl;

    std::vector<int> facets_list;
    std::vector<int> left_facets;
    std::vector<int> right_facets;

    // if ray intersects bounding box, else no intersection to check
    if (ray_box_intersection(ray_origin, ray_direction, cur_root.bbox)) {

        //std::cout << "In search_tree: ray intersects with: cur_root = triangle:" << cur_root.triangle << ", left: " << cur_root.left << ", right: " << cur_root.right << std::endl;

        // if leaf, append facet id
        if (cur_root.left == -1 && cur_root.right == -1) {
            if (cur_root.triangle >= 0) { // safety check that triangle index is valid
                facets_list.push_back(cur_root.triangle); // add facet id index to list
                //std::cout << "added trianlge to facets_list: index = " << cur_root.triangle << std::endl;
            }
        }
        else {
            // if internal node, recursively check left child
            if (cur_root.left >= 0) {
                AABBTree::Node cur_left_child = bvh.nodes[cur_root.left];

                left_facets = search_tree(ray_origin, ray_direction, cur_left_child);
            }

            // if internal node, recursively check right child
            if (cur_root.right >= 0) {
                AABBTree::Node cur_right_child = bvh.nodes[cur_root.right];

                right_facets = search_tree(ray_origin, ray_direction, cur_right_child);
            }

            // add newly found facets to list of intersecting facets to return
            facets_list.insert(facets_list.end(), left_facets.begin(), left_facets.end());
            facets_list.insert(facets_list.end(), right_facets.begin(), right_facets.end());
        }

    }

    return facets_list; // will be empty if no intersection found
}

//Finds the closest intersecting object returns its index
//In case of intersection it writes into p and N (intersection point and normals)
bool find_nearest_object(const Vector3d &ray_origin, const Vector3d &ray_direction, Vector3d &p, Vector3d &N)
{

    // Brute force search
    // Method (1): Traverse every triangle and return the closest hit.

    // Find the object in the scene that intersects the ray first
    // we store the index and the 'closest_t' to their expected values
    int closest_index = -1;
    double closest_t = std::numeric_limits<double>::max(); //closest t is "+ infinity"

    Vector3d tmp_p, tmp_N;

    /// START OF METHOD 1 ///

    /*
    //std::cout << "Using Method 1: Brute force." << std::endl;

    // find closest facet (using brute force)
    for (int i = 0; i < facets.rows(); ++i)
    {

        // get vertices for the current triangle

        //std::cout << "facet(" << i << ", 0) = " << facets(i, 0) << std::endl;

        const Vector3d A = vertices.row(facets(i, 0));
        const Vector3d B = vertices.row(facets(i, 1));
        const Vector3d C = vertices.row(facets(i, 2));

        //std::cout << "i: " << i << " A = " << A << ", B = " << B << ", C = " << C << std::endl; 
        //std::cout << "i = " << i << "facets.rows() = " << facets.rows() << std::endl;

        //returns t and writes on tmp_p and tmp_N
        const double t = ray_triangle_intersection(ray_origin, ray_direction, A, B, C, tmp_p, tmp_N);

        //We have intersection
        if (t >= 0)
        {
            //The point is before our current closest t
            if (t < closest_t)
            {
                closest_index = i;
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
        }
    }
    */

    /// END OF METHOD 1 ///



    /// START OF METHOD 2 ///

    
    //std::cout << "Using Method 2: AABBTree." << std::endl;

    // Use the BVH tree
    // Method (2): Traverse the BVH tree and test the intersection with a
    // triangles at the leaf nodes that intersects the input ray.

    AABBTree::Node root_node = bvh.nodes[bvh.root]; // returns the node for the main tree root

    // this checks the tree node bounding boxes and returns a list
    // of only facets indexes where the bounding box of that triangle intersects with the ray.
    // we then process this reduced list in the same way that we do for the brute force method,
    // only this time, the list is significantly reduced in size (likely often only 1 or 2 facets).
    std::vector<int> tree_facet_ids = search_tree(ray_origin, ray_direction, root_node);

    //std::cout << "after search_tree() done, tree_facet_ids.size() = " << tree_facet_ids.size() << std::endl;

    // find closest facet (using the tree)
    for (int i = 0; i < tree_facet_ids.size(); ++i)
    {
        // get vertices for the current triangle

        //std::cout << "facet(" << facets(tree_facet_ids[i] << ", 0) = " << facets(facets(tree_facet_ids[i], 0) << std::endl;

        const Vector3d A = vertices.row(facets(tree_facet_ids[i], 0));
        const Vector3d B = vertices.row(facets(tree_facet_ids[i], 1));
        const Vector3d C = vertices.row(facets(tree_facet_ids[i], 2));

        //std::cout << "i: " << i << " A = " << A << ", B = " << B << ", C = " << C << std::endl; 
        //std::cout << "i = " << i << "facets.rows() = " << facets.rows() << std::endl;

        //returns t and writes on tmp_p and tmp_N
        const double t = ray_triangle_intersection(ray_origin, ray_direction, A, B, C, tmp_p, tmp_N);

        //We have intersection
        if (t >= 0)
        {
            //The point is before our current closest t
            if (t < closest_t)
            {
                closest_index = i;
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
        }
    }
    

    /// END OF METHOD 2 ///



    // NOTE: To keep things simple, we will still use brute force for spheres and paralleloograms
    // since we have a much more manageable amount of them in our scene.
    // Therefore, don't comment out this code when using either method.
    
    for (int i = 0; i < sphere_centers.size(); ++i)
    {
        //returns t and writes on tmp_p and tmp_N
        const double t = ray_sphere_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);
        //We have intersection
        if (t >= 0)
        {
            //The point is before our current closest t
            if (t < closest_t)
            {
                closest_index = i;
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
        }
    }


    // compute closest pgram if closer than the closest facet
    for (int i = 0; i < parallelograms.size(); ++i)
    {
        //returns t and writes on tmp_p and tmp_N
        const double t = ray_parallelogram_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);

        //We have intersection
        if (t >= 0) //original code
        {
            //The point is before our current closest t
            if (t < closest_t)
            {
                //closest_index = sphere_centers.size() + i;
                closest_index = 100; // since we don't care about the index in Assignment 4, simple fix to make it non-dependent on spheres
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
        }
    }


    // return if a closest object of any kind found
    if (closest_index < 0) {
        return false;
    }
    else {
        return true;
    }

}

////////////////////////////////////////////////////////////////////////////////
// Raytracer code
////////////////////////////////////////////////////////////////////////////////

//Checks if the light is visible
bool is_light_visible(const Vector3d& ray_origin, const Vector3d& ray_direction, const Vector3d& light_position)
{
    // find_nearest_object will return the intersection point and normal if an object is hit
    Vector3d p, N;
    bool intersects_object = false;

    //cast ray slightly above actual origin to avoid intersection with itself (see textbook p.94)
    intersects_object = find_nearest_object(ray_origin + (ray_direction * 0.0001), ray_direction, p, N); // using the epsilon value recommended in class

    // intersected with an object which blocks the light. Therefore light is not visible.
    if (intersects_object) {
        // if distance to intersection point is further than distance to light, the object is behind the light (and therefore doesn't block the light)
        bool is_behind_light = (p - ray_origin).norm() > (light_position - ray_origin).norm();

        if (is_behind_light) {
            return true;
        }
        return false;
    }
    // nothing intersected with ray. Therefore light is visible.
    else {
        return true;
    }
    
}


Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction, int max_bounce)
{
    //Intersection point and normal, these are output of find_nearest_object
    Vector3d p, N;

    const bool nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    if (!nearest_object)
    {
        // Return a transparent color
        return Vector4d(0, 0, 0, 0);
    }

    // Ambient light contribution
    const Vector4d ambient_color = obj_ambient_color.array() * ambient_light.array();

    // Punctual lights contribution (direct lighting)
    Vector4d lights_color(0, 0, 0, 0);
    for (int i = 0; i < light_positions.size(); ++i)
    {
        const Vector3d &light_position = light_positions[i];
        const Vector4d &light_color = light_colors[i];

        Vector4d diff_color = obj_diffuse_color;

        const Vector3d Li = (light_position - p).normalized();

        // Shadow/Light contribution: Shoot a shadow ray to determine if the light should affect the intersection point and call is_light_visible()
        if (is_light_visible(p, Li, light_position) == false) {
            continue; // if light not visible, it doesn't contribute, therefore we skip all computations below related to its contribution.
        }
        

        // Diffuse contribution
        //const Vector3d Li = (light_position - p).normalized();
        const Vector4d diffuse = diff_color * std::max(Li.dot(N), 0.0);

        // Specular contribution
        const Vector3d Hi = (Li - ray_direction).normalized();
        const Vector4d specular = obj_specular_color * std::pow(std::max(N.dot(Hi), 0.0), obj_specular_exponent);
        // Vector3d specular(0, 0, 0);

        // Attenuate lights according to the squared distance to the lights
        const Vector3d D = light_position - p;

        // Use all contributions to determine light_color
        lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
    }

    // Compute the color of the reflected ray and add its contribution to the current point color.
    // use refl_color

    Vector4d refl_color = obj_reflection_color;

    Vector4d reflection_color(0, 0, 0, 0); // initiate to all zeros, then for each bounce, add onto it

    
    if (max_bounce >= 0) { // recursively shoot reflection rays. Stop after 5 bounces.
        Vector3d reflection_ray = -(2 * N * (ray_direction.dot(N)) - ray_direction);

        reflection_color = reflection_color + refl_color.cwiseProduct(shoot_ray((p + (reflection_ray * 0.0001)), reflection_ray, max_bounce - 1));
        // again, like shadows, we need an epsilon offset for reflection rays to avoid a grainy image.
    }
    

    // Rendering equation
    Vector4d C = ambient_color + lights_color + reflection_color;

    //Set alpha to 1
    C(3) = 1;

    return C;
}

////////////////////////////////////////////////////////////////////////////////

void raytrace_scene()
{
    std::cout << "Simple ray tracer." << std::endl;

    int w = 640;
    int h = 480;
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers an viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);

    // code from assignment 3
    double image_y = tan((field_of_view / 2)) * focal_length;
    double image_x = aspect_ratio * image_y;

    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    const Vector3d image_origin(-image_x, image_y, camera_position[2] - focal_length);
    const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
    const Vector3d y_displacement(0, -2.0 / h * image_y, 0);

    for (unsigned i = 0; i < w; ++i)
    {
        for (unsigned j = 0; j < h; ++j)
        {
            const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

            // Prepare the ray
            Vector3d ray_origin;
            Vector3d ray_direction;

            if (is_perspective)
            {
                // Perspective camera
                ray_origin = camera_position;
                ray_direction = (pixel_center - camera_position).normalized();
            }
            else
            {
                // Orthographic camera
                ray_origin = pixel_center;
                ray_direction = Vector3d(0, 0, -1);
            }

            const Vector4d C = shoot_ray(ray_origin, ray_direction, max_bounce);
            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = C(3);
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    setup_scene();
    
    raytrace_scene();

    return 0;
}
