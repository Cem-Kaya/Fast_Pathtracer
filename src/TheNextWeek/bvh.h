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


    // ==================================================================================================================================================
   // OUR CODE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   // takes a list of hitables liek triangle list and makes a bvh from that 
    bvh_node(const hittable_list& list) {
        // Extract all hittables from the list (including nested hittable_lists)
        std::vector<shared_ptr<hittable>> objects;
        for (const auto& object : list.objects) {
            if (auto obj_list = std::dynamic_pointer_cast<hittable_list>(object)) {
                objects.insert(objects.end(), obj_list->objects.begin(), obj_list->objects.end());
            }
            else {
                objects.push_back(object);
            }
        }

        // **Call the main constructor**
        *this = bvh_node(objects, 0, objects.size(), "sah"); // "mid"  or "sah" split option
    }


    bvh_node(std::vector<shared_ptr<hittable>>& objects, size_t start, size_t end, const std::string& split_type) {
        // Build the bounding box of the span of source objects.
        bbox = aabb::empty;
        for (size_t object_index = start; object_index < end; object_index++)
            bbox = aabb(bbox, objects[object_index]->bounding_box());

        size_t object_span = end - start;

        if (object_span == 1) {
            left = right = objects[start];
        }
        else if (object_span == 2) {
            left = objects[start];
            right = objects[start + 1];
        }
        else {
            if (split_type == "mid") {
                // **MIDPOINT SPLIT**
                int axis = bbox.longest_axis();
                auto comparator = (axis == 0) ? box_x_compare
                    : (axis == 1) ? box_y_compare
                    : box_z_compare;

                std::sort(objects.begin() + start, objects.begin() + end, comparator);

                auto mid = start + object_span / 2;
                left = make_shared<bvh_node>(objects, start, mid, split_type);
                right = make_shared<bvh_node>(objects, mid, end, split_type);
            }
            else if (split_type == "sah") {
                // **SAH SPLIT**
                int best_axis = -1;
                float best_cost = std::numeric_limits<float>::infinity();
                size_t best_split_index = start;

                for (int axis = 0; axis < 3; axis++) {
                    std::sort(objects.begin() + start, objects.begin() + end,
                        [axis](const shared_ptr<hittable>& a, const shared_ptr<hittable>& b) {
                            return a->bounding_box().centroid()[axis] < b->bounding_box().centroid()[axis];
                        });

                    std::vector<aabb> left_bboxes(object_span);
                    std::vector<aabb> right_bboxes(object_span);

                    aabb left_bbox = aabb::empty;
                    for (size_t i = start; i < end; ++i) {
                        left_bbox = aabb(left_bbox, objects[i]->bounding_box());
                        left_bboxes[i - start] = left_bbox;
                    }

                    aabb right_bbox = aabb::empty;
                    for (size_t i = end - 1; i >= start; --i) {
                        right_bbox = aabb(right_bbox, objects[i]->bounding_box());
                        right_bboxes[i - start] = right_bbox;
                    }

                    for (size_t i = start + 1; i < end; ++i) {
                        int num_left = i - start;
                        int num_right = end - i;

                        aabb left_bbox = left_bboxes[i - start - 1];
                        aabb right_bbox = right_bboxes[i - start];

                        float parent_surface_area = left_bbox.surface_area() + right_bbox.surface_area();
                        float cost = 1 +
                            (left_bbox.surface_area() / parent_surface_area) * num_left +
                            (right_bbox.surface_area() / parent_surface_area) * num_right;

                        if (cost < best_cost) {
                            best_cost = cost;
                            best_axis = axis;
                            best_split_index = i;
                        }
                    }
                }

                std::sort(objects.begin() + start, objects.begin() + end,
                    [best_axis](const shared_ptr<hittable>& a, const shared_ptr<hittable>& b) {
                        return a->bounding_box().centroid()[best_axis] < b->bounding_box().centroid()[best_axis];
                    });

                auto mid = best_split_index;
                left = make_shared<bvh_node>(objects, start, mid, split_type);
                right = make_shared<bvh_node>(objects, mid, end, split_type);
            }
            else {
                throw std::invalid_argument("Unknown split type: " + split_type);
            }
        }

        bbox = aabb(left->bounding_box(), right->bounding_box());
    }


   
    bool hit(const ray& r, interval ray_t, hit_record& rec, ray_state& state) const override {
        // We are traversing a BVH node, increment the traversal step counter
        state.traversal_steps++;



        if (!bbox.hit(r, ray_t))
            return false;

        bool hit_left = left->hit(r, ray_t, rec, state);
        bool hit_right = right->hit(r, interval(ray_t.min, hit_left ? rec.t : ray_t.max), rec, state);

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

    static bool box_x_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
        return box_compare(a, b, 0);
    }

    static bool box_y_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
        return box_compare(a, b, 1);
    }

    static bool box_z_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
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




#endif