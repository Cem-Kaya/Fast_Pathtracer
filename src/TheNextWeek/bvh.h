#ifndef BVH_H
#define BVH_H
//==============================================================================================
// Originally written in 2016 by Peter Shirley <ptrshrl@gmail.com>
//
// To the extent possible under law, the author(s) have dedicated all copyright and related and
// neighboring rights to this software to the public domain worldwide. This software is
// distributed without any warranty.
//
// You should have received a copy (see file COPYING.txt) of the CC0 Public Domain Dedication
// along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
//==============================================================================================

#include "aabb.h"
#include "hittable.h"
#include "hittable_list.h"

#include <algorithm>
#include <vector>
#include <memory>
#include <unordered_map>


struct pixel_efficiency_data {
    color accumulated_color;
    int total_traversal_steps;
    int total_intersection_tests;

    pixel_efficiency_data()
        : accumulated_color(color(0, 0, 0)),
        total_traversal_steps(0),
        total_intersection_tests(0) {
    }
};








class bvh_node : public hittable {
  public:
   

    bvh_node(std::vector<shared_ptr<hittable>>& objects, size_t start, size_t end) {
        // Build the bounding box of the span of source objects.
        bbox = aabb::empty;
        for (size_t object_index=start; object_index < end; object_index++)
            bbox = aabb(bbox, objects[object_index]->bounding_box());

        int axis = bbox.longest_axis();

        auto comparator = (axis == 0) ? box_x_compare
                        : (axis == 1) ? box_y_compare
                                      : box_z_compare;

        size_t object_span = end - start;

        if (object_span == 1) {
            left = right = objects[start];
        } else if (object_span == 2) {
            left = objects[start];
            right = objects[start+1];
        } else {
            std::sort(std::begin(objects) + start, std::begin(objects) + end, comparator);

            auto mid = start + object_span/2;
            left = make_shared<bvh_node>(objects, start, mid);
            right = make_shared<bvh_node>(objects, mid, end);
        }
    }

    // ==================================================================================================================================================
    // OUR CODE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // takes a list of hitables liek triangle list and makes a bvh from that 
    bvh_node(const hittable_list& list) {
        bbox = aabb::empty;

        // Extract all hittables 
        std::vector<shared_ptr<hittable>> objects;
        for (const auto& object : list.objects) {
            // If the object is a list itself, add its elements
            if (auto obj_list = std::dynamic_pointer_cast<hittable_list>(object)) {
                objects.insert(objects.end(), obj_list->objects.begin(), obj_list->objects.end());
            }
            else {
                objects.push_back(object);
            }
        }

        // Now build the BVH over the extracted objects
        int axis = bbox.longest_axis();
        auto comparator = (axis == 0) ? box_x_compare
            : (axis == 1) ? box_y_compare
            : box_z_compare;

        int object_span = objects.size();

        if (object_span == 1) {
            left = right = objects[0];
        }
        else if (object_span == 2) {
            left = objects[0];
            right = objects[1];
        }
        else {
            std::sort(std::begin(objects), std::end(objects), comparator);

            auto mid = object_span / 2;
            left = make_shared<bvh_node>(objects, 0, mid);
            right = make_shared<bvh_node>(objects, mid, object_span);
        }
    }


    bool hit(const ray& r, interval ray_t, hit_record& rec, ray_state& state) const override {
        // We are traversing a BVH node, increment the traversal step counter
        state.traversal_steps++;



        if (!bbox.hit(r, ray_t))
            return false;

        bool hit_left = left->hit(r, ray_t, rec, state );
        bool hit_right = right->hit(r, interval(ray_t.min, hit_left ? rec.t : ray_t.max), rec, state );

        return hit_left || hit_right;
    }

    aabb bounding_box() const override { return bbox; }

  private:
    shared_ptr<hittable> left;
    shared_ptr<hittable> right;
    aabb bbox;

    static bool box_compare(
        const shared_ptr<hittable> a, const shared_ptr<hittable> b, int axis_index
    ) {
        auto a_axis_interval = a->bounding_box().axis_interval(axis_index);
        auto b_axis_interval = b->bounding_box().axis_interval(axis_index);
        return a_axis_interval.min < b_axis_interval.min;
    }

    static bool box_x_compare (const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
        return box_compare(a, b, 0);
    }

    static bool box_y_compare (const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
        return box_compare(a, b, 1);
    }

    static bool box_z_compare (const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
        return box_compare(a, b, 2);
    }
};


/*  bugy code refactorer ing broke it and will fix it soon 
class octree_node : public hittable {
public:
    aabb bbox;
    std::vector<std::unique_ptr<octree_node>> children;
    std::vector<shared_ptr<hittable>> objects;  // This is the member variable for storing hittables

    // Constructor 1: Build octree from a list of hittables
    octree_node(const hittable_list& list) {
        // This constructor processes a list and then delegates to the main constructor
        this->objects.clear();  // Clear existing objects if any
        for (const auto& object : list.objects) {
            if (auto obj_list = std::dynamic_pointer_cast<hittable_list>(object)) {
                this->objects.insert(this->objects.end(), obj_list->objects.begin(), obj_list->objects.end());
            }
            else {
                this->objects.push_back(object);
            }
        }

        // Call the main constructor with processed objects
        *this = octree_node(this->objects, aabb::compute_bounding_box(this->objects), 0);
    }

    // Main Constructor: Splits objects into octants
    octree_node(const std::vector<shared_ptr<hittable>>& objects, const aabb& bound, int depth)
        : bbox(bound), objects(objects)  // Initialize members directly
    {
        if (objects.size() <= 2 || depth > max_depth) {
            // Base case: few objects or max depth reached
            return;
        }

        std::vector<std::vector<shared_ptr<hittable>>> children_objects(8);
        vec3 center = this->bbox.centroid();

        for (const auto& object : objects) {
            int child_index = 0;
            vec3 obj_center = object->bounding_box().centroid();
            if (obj_center.x > center.x) child_index |= 1;
            if (obj_center.y > center.y) child_index |= 2;
            if (obj_center.z > center.z) child_index |= 4;

            children_objects[child_index].push_back(object);
        }

        children.resize(8);  // Ensure there is space for 8 children

        for (int i = 0; i < 8; ++i) {
            if (!children_objects[i].empty()) {
                vec3 child_min = compute_child_bbox_min(i, this->bbox);
                vec3 child_max = compute_child_bbox_max(i, this->bbox);
                children[i] = std::make_unique<octree_node>(children_objects[i], aabb(child_min, child_max), depth + 1);
            }
        }
    }

    // Hit method to test if a ray hits the octree
    bool hit(const ray& r, interval ray_t, hit_record& rec, ray_state& state) const override {
        state.traversal_steps++;

        // Check if ray hits this node's bounding box
        if (!bbox.hit(r, ray_t))
            return false;

        bool hit_anything = false;

        // Check child nodes
        for (const auto& child : children) {
            if (child && child->hit(r, ray_t, rec, state)) {
                hit_anything = true;
                ray_t.max = rec.t; // Narrow the search window
            }
        }

        // Check objects in this node
        for (const auto& object : objects) {
            if (object->hit(r, ray_t, rec, state)) {
                hit_anything = true;
                ray_t.max = rec.t;
            }
        }

        return hit_anything;
    }

    // Return the bounding box of this node
    aabb bounding_box() const override {
        return bbox;
    }

private:
    // Compute the min point of the bounding box for the i-th child
    vec3 compute_child_bbox_min(int index, const aabb& parent_bbox) const {
        vec3 c = parent_bbox.centroid();
        vec3 mn = parent_bbox.min();
        // Adjust min depending on the octant index
        if (index & 1) mn.x = c.x; // Right half
        if (index & 2) mn.y = c.y; // Upper half
        if (index & 4) mn.z = c.z; // Front half
        return mn;
    }

    // Compute the max point of the bounding box for the i-th child
    vec3 compute_child_bbox_max(int index, const aabb& parent_bbox) const {
        vec3 c = parent_bbox.centroid();
        vec3 mx = parent_bbox.max();
        // Adjust max depending on the octant index
        if (!(index & 1)) mx.x = c.x; // Left half
        if (!(index & 2)) mx.y = c.y; // Lower half
        if (!(index & 4)) mx.z = c.z; // Back half
        return mx;
    }

    static constexpr int max_depth = 10; // Limit recursion

};

*/

// Grid Acceleration Structure
class grid_acceleration : public hittable {
public:
    // Constructor for a list of hittables
    grid_acceleration(const hittable_list& list, int cells_per_axis) {
        bounds = aabb::empty;

        // Extract all hittables
        std::vector<std::shared_ptr<hittable>> objects;
        for (const auto& object : list.objects) {
            // If the object is a list itself, add its elements
            if (auto obj_list = std::dynamic_pointer_cast<hittable_list>(object)) {
                objects.insert(objects.end(), obj_list->objects.begin(), obj_list->objects.end());
            }
            else {
                objects.push_back(object);
            }
        }

        // Compute overall bounds and initialize the grid
        for (const auto& obj : objects) {
            bounds = aabb(bounds, obj->bounding_box());
        }

        initialize_grid(objects, cells_per_axis);
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec, ray_state& state) const override {
        state.traversal_steps++; // Increment traversal steps

        if (!bounds.hit(r, ray_t)) return false;

        vec3 inv_dir = vec3(1.0 / r.direction().x(), 1.0 / r.direction().y(), 1.0 / r.direction().z());
        vec3 t_min = (bounds.min() - r.origin()) * inv_dir;
        vec3 t_max = (bounds.max() - r.origin()) * inv_dir;

        double t_entry = std::max(std::max(std::min(t_min.x(), t_max.x()), std::min(t_min.y(), t_max.y())), std::min(t_min.z(), t_max.z()));
        double t_exit = std::min(std::min(std::max(t_min.x(), t_max.x()), std::max(t_min.y(), t_max.y())), std::max(t_min.z(), t_max.z()));

        if (t_exit <= t_entry || t_exit < ray_t.min || t_entry > ray_t.max) return false;

        vec3 grid_size = bounds.size() / static_cast<double>(cells_per_axis);
        vec3 inv_grid_size = vec3(1.0 / grid_size.x(), 1.0 / grid_size.y(), 1.0 / grid_size.z());
        point3 entry_point = r.origin() + t_entry * r.direction();
        vec3 cell = (entry_point - bounds.min()) * inv_grid_size;
        vec3 step = vec3(r.direction().x() > 0 ? 1 : -1,
            r.direction().y() > 0 ? 1 : -1,
            r.direction().z() > 0 ? 1 : -1);
        vec3 t_delta = step * grid_size * inv_dir;
        vec3 next_t = vec3(step.x() > 0 ? std::ceil(cell.x()) : std::floor(cell.x()),
            step.y() > 0 ? std::ceil(cell.y()) : std::floor(cell.y()),
            step.z() > 0 ? std::ceil(cell.z()) : std::floor(cell.z())) * t_delta;

        while (t_entry <= t_exit) {
            int cell_index = compute_cell_index(cell);
            if (cell_index >= 0 && cell_index < cells.size()) {
                for (const auto& obj : cells[cell_index]) {
                    if (obj->hit(r, ray_t, rec, state)) return true;
                }
            }

            if (next_t.x() < next_t.y() && next_t.x() < next_t.z()) {
                t_entry = next_t.x();
                cell[0] += step.x();
                next_t[0] += t_delta.x();
            }
            else if (next_t.y() < next_t.z()) {
                t_entry = next_t.y();
                cell[1] += step.y();
                next_t[1] += t_delta.y();
            }
            else {
                t_entry = next_t.z();
                cell[2] += step.z();
                next_t[2] += t_delta.z();
            }
        }
        return false;
    }

    aabb bounding_box() const override { return bounds; }

private:
    aabb bounds;
    int cells_per_axis;
    std::vector<std::vector<std::shared_ptr<hittable>>> cells;

    void initialize_grid(std::vector<std::shared_ptr<hittable>>& objects, int cells_per_axis) {
        this->cells_per_axis = cells_per_axis;
        int total_cells = cells_per_axis * cells_per_axis * cells_per_axis;
        cells.resize(total_cells);

        vec3 grid_size = bounds.size() / static_cast<double>(cells_per_axis);

        for (const auto& obj : objects) {
            aabb obj_bounds = obj->bounding_box();
            vec3 min_cell = (obj_bounds.min() - bounds.min()) / grid_size;
            vec3 max_cell = (obj_bounds.max() - bounds.min()) / grid_size;

            for (int x = std::floor(min_cell.x()); x <= std::ceil(max_cell.x()); x++) {
                for (int y = std::floor(min_cell.y()); y <= std::ceil(max_cell.y()); y++) {
                    for (int z = std::floor(min_cell.z()); z <= std::ceil(max_cell.z()); z++) {
                        int cell_index = compute_cell_index(vec3(x, y, z));
                        if (cell_index >= 0 && cell_index < total_cells) {
                            cells[cell_index].push_back(obj);
                        }
                    }
                }
            }
        }
    }

    int compute_cell_index(const vec3& cell) const {
        return static_cast<int>(cell.x()) +
            static_cast<int>(cell.y()) * cells_per_axis +
            static_cast<int>(cell.z()) * cells_per_axis * cells_per_axis;
    }
};


#endif
