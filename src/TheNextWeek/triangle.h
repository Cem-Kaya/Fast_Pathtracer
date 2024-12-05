#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "hittable.h"
#include "hittable_list.h"
#include "material.h"
#include "interval.h"
#include "aabb.h"
#include "vec3.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#include <iostream>
#include <algorithm>
#include <memory>

class triangle : public hittable {
public:
    triangle(const point3& v0, const point3& v1, const point3& v2, shared_ptr<material> mat)
        : v0(v0), v1(v1), v2(v2), mat(mat)
    {
        // Compute the normal using cross product
        normal = unit_vector(cross(v1 - v0, v2 - v0));
        set_bounding_box();
    }

    virtual void set_bounding_box() {
        // Compute the bounding box of the triangle
        auto min_point = point3(
            std::fmin(v0.x(), std::fmin(v1.x(), v2.x())),
            std::fmin(v0.y(), std::fmin(v1.y(), v2.y())),
            std::fmin(v0.z(), std::fmin(v1.z(), v2.z()))
        );

        auto max_point = point3(
            std::fmax(v0.x(), std::fmax(v1.x(), v2.x())),
            std::fmax(v0.y(), std::fmax(v1.y(), v2.y())),
            std::fmax(v0.z(), std::fmax(v1.z(), v2.z()))
        );

        bbox = aabb(min_point, max_point);
    }

    aabb bounding_box() const override { return bbox; }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        // Möller-Trumbore intersection algorithm
        const double EPSILON = 1e-8;
        vec3 edge1 = v1 - v0;
        vec3 edge2 = v2 - v0;
        vec3 h = cross(r.direction(), edge2);
        double a = dot(edge1, h);

        if (fabs(a) < EPSILON)
            return false; // Parallel to the triangle plane

        double f = 1.0 / a;
        vec3 s = r.origin() - v0;
        double u = f * dot(s, h);

        if (u < 0.0 || u > 1.0)
            return false;

        vec3 q = cross(s, edge1);
        double v = f * dot(r.direction(), q);

        if (v < 0.0 || u + v > 1.0)
            return false;

        double t = f * dot(edge2, q);

        if (!ray_t.contains(t))
            return false;

        // Set hit record
        rec.t = t;
        rec.p = r.at(t);
        rec.normal = normal;
        rec.mat = mat;
        rec.set_face_normal(r, normal);
        rec.u = u;
        rec.v = v;

        return true;
    }

private:
    point3 v0, v1, v2;
    vec3 normal;
    shared_ptr<material> mat;
    aabb bbox;
};

// Function to parse .obj files
inline shared_ptr<hittable_list> parse_obj(const std::string& filename, shared_ptr<material> mat) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + filename);
    }

    std::vector<point3> vertices;
    auto triangles = make_shared<hittable_list>();

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream stream(line);
        std::string prefix;
        stream >> prefix;

        if (prefix == "v") {
            // Vertex
            double x, y, z;
            stream >> x >> y >> z;
            vertices.emplace_back(x, y, z);

            // Debug: Print the vertex and current size of vertices
            //std::cout << "Loaded vertex: (" << x << ", " << y << ", " << z << ")" << std::endl;
            //std::cout << "Vertices size: " << vertices.size() << std::endl;
        }
        else if (prefix == "f") {
            try {
                std::string vertex1, vertex2, vertex3;
                stream >> vertex1 >> vertex2 >> vertex3;

                auto parse_index = [](const std::string& v) {
                    size_t slash_pos = v.find('/');
                    return std::stoi(v.substr(0, slash_pos)) - 1;
                    };

                int v0_idx = parse_index(vertex1);
                int v1_idx = parse_index(vertex2);
                int v2_idx = parse_index(vertex3);

                // Debug: Print indices
                //std::cout << "Face indices: " << v0_idx << ", " << v1_idx << ", " << v2_idx << std::endl;
                //std::cout << "Vertices size: " << vertices.size() << std::endl;

                // Boundary check
                if (v0_idx < 0 || v0_idx >= vertices.size() ||
                    v1_idx < 0 || v1_idx >= vertices.size() ||
                    v2_idx < 0 || v2_idx >= vertices.size()) {
                    throw std::runtime_error("Face index out of bounds in " + filename);
                }

                // Add triangle
                triangles->add(make_shared<triangle>(
                    vertices[v0_idx], vertices[v1_idx], vertices[v2_idx], mat
                ));
            }
            catch (const std::exception& e) {
                std::cerr << "Error parsing face in file " << filename << ": " << e.what() << std::endl;
            }
        }


    }

    return triangles;
}

#endif
