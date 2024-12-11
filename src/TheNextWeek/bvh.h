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
        *this = bvh_node(objects, 0, objects.size(), "mid"); // "mid"  or "sah" split option
    }

    // actual constructers
    bvh_node(std::vector<shared_ptr<hittable>>& objects, size_t start, size_t end, const std::string& split_type) {
        // Build the bounding box of the span of source objects.
        bbox = aabb::empty;
        for (int object_index = start; object_index < end; object_index++)
            bbox = aabb(bbox, objects[object_index]->bounding_box());

        int object_span = end - start;

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
                
                //best split 
                int best_split_index = start;

                for (int axis = 0; axis < 3; axis++) {
                    std::sort(objects.begin() + start, objects.begin() + end,
                        [axis](const shared_ptr<hittable>& a, const shared_ptr<hittable>& b) {
                            return a->bounding_box().centroid()[axis] < b->bounding_box().centroid()[axis];
                        });

                    std::vector<aabb> left_bboxes(object_span);
                    std::vector<aabb> right_bboxes(object_span);

                    aabb left_bbox = aabb::empty;
                    for (int i = start; i < end; ++i) {
                        left_bbox = aabb(left_bbox, objects[i]->bounding_box());
                        left_bboxes[i - start] = left_bbox;
                    }

                    aabb right_bbox = aabb::empty;
                    for (int i = end - 1; i >= start; --i) {
                        right_bbox = aabb(right_bbox, objects[i]->bounding_box());
                        right_bboxes[i - start] = right_bbox;
                    }

                    for (int i = start + 1; i < end; ++i) {
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

/////////////////////////




class octree_node : public hittable {
public:
    aabb bbox;
    std::vector<std::unique_ptr<octree_node>> children;
    std::vector<shared_ptr<hittable>> objects;

    octree_node(const hittable_list& list) {
        this->objects.clear();
        for (const auto& object : list.objects) {
            if (auto obj_list = std::dynamic_pointer_cast<hittable_list>(object)) {
                this->objects.insert(this->objects.end(), obj_list->objects.begin(), obj_list->objects.end());
            }
            else {
                this->objects.push_back(object);
            }
        }

        this->bbox = aabb::empty;
        for (const auto& object : this->objects) {
            this->bbox = aabb(this->bbox, object->bounding_box());
        }

        *this = octree_node(this->objects, this->bbox, 0);
    }

    octree_node(const std::vector<shared_ptr<hittable>>& objects, const aabb& bound, int depth)
        : bbox(bound), objects(objects)
    {
        if (objects.size() <= 2 || depth > max_depth) {
            return;
        }

        std::vector<std::vector<shared_ptr<hittable>>> children_objects(8);
        vec3 center = this->bbox.centroid();

        for (const auto& object : objects) {
            int child_index = 0;
            vec3 obj_center = object->bounding_box().centroid();
            if (obj_center.x() > center.x()) child_index |= 1;
            if (obj_center.y() > center.y()) child_index |= 2;
            if (obj_center.z() > center.z()) child_index |= 4;

            children_objects[child_index].push_back(object);
        }

        children.resize(8);

        for (int i = 0; i < 8; ++i) {
            if (!children_objects[i].empty()) {
                vec3 child_min = compute_child_bbox_min(i, this->bbox);
                vec3 child_max = compute_child_bbox_max(i, this->bbox);
                children[i] = std::make_unique<octree_node>(children_objects[i], aabb(child_min, child_max), depth + 1);
            }
        }
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec, ray_state& state) const override {
        state.traversal_steps++;

        if (!bbox.hit(r, ray_t))
            return false;

        bool hit_anything = false;

        for (const auto& child : children) {
            if (child && child->hit(r, ray_t, rec, state)) {
                hit_anything = true;
                ray_t.max = rec.t;
            }
        }

        for (const auto& object : objects) {
            if (object->hit(r, ray_t, rec, state)) {
                hit_anything = true;
                ray_t.max = rec.t;
            }
        }

        return hit_anything;
    }

    aabb bounding_box() const override {
        return bbox;
    }

private:
    vec3 compute_child_bbox_min(int index, const aabb& parent_bbox) const {
        vec3 mn = vec3(parent_bbox.x.max, parent_bbox.y.max, parent_bbox.z.max); // Initialize with max

        // Adjust min depending on the octant index
        if (!(index & 1)) mn[0] = parent_bbox.x.min;  // Left half
        if (!(index & 2)) mn[1] = parent_bbox.y.min;  // Lower half
        if (!(index & 4)) mn[2] = parent_bbox.z.min;  // Back half
        return mn;
    }


    vec3 compute_child_bbox_max(int index, const aabb& parent_bbox) const {
        vec3 mx = vec3(parent_bbox.x.min, parent_bbox.y.min, parent_bbox.z.min); // Initialize with min

        // Adjust max depending on the oct index
        if (index & 1) mx[0] = parent_bbox.x.max;  // Right half
        if (index & 2) mx[1] = parent_bbox.y.max;  // Upp half
        if (index & 4) mx[2] = parent_bbox.z.max;  // Front half 
        return mx;
    }
    static constexpr int max_depth = 32;
};


////////////////////////////////////////////////////////////////////////

class grid_acceleration : public hittable {
public:
    grid_acceleration(const hittable_list& list, int cells_per_axis = 100) {
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

        // Compute the big bounding box
        bbox = aabb::empty;
        for (const auto& object : objects) {
            bbox = aabb(bbox, object->bounding_box());
        }

        // Initialize grid parameters
        this->cells_per_axis = cells_per_axis;
        cell_size = vec3(bbox.x.max - bbox.x.min, bbox.y.max - bbox.y.min, bbox.z.max - bbox.z.min) / cells_per_axis;

        // Create grid cells
        cells.resize(cells_per_axis * cells_per_axis * cells_per_axis);

        // Assign objects to grid cells
        for (const auto& object : objects) {
            aabb obj_bbox = object->bounding_box();

            vec3 min_corner = vec3(
                (obj_bbox.x.min - bbox.x.min) / cell_size.x(),
                (obj_bbox.y.min - bbox.y.min) / cell_size.y(),
                (obj_bbox.z.min - bbox.z.min) / cell_size.z()
            );
            vec3 max_corner = vec3(
                (obj_bbox.x.max - bbox.x.min) / cell_size.x(),
                (obj_bbox.y.max - bbox.y.min) / cell_size.y(),
                (obj_bbox.z.max - bbox.z.min) / cell_size.z()
            );

            for (int x = static_cast<int>(min_corner.x()); x <= static_cast<int>(max_corner.x()); ++x) {
                for (int y = static_cast<int>(min_corner.y()); y <= static_cast<int>(max_corner.y()); ++y) {
                    for (int z = static_cast<int>(min_corner.z()); z <= static_cast<int>(max_corner.z()); ++z) {
                        int index = x + y * cells_per_axis + z * cells_per_axis * cells_per_axis;
                        if (index >= 0 && index < cells.size()) {
                            cells[index].push_back(object);
                        }
                    }
                }
            }
        }
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec, ray_state& state) const override {
        state.traversal_steps++;

        if (!bbox.hit(r, ray_t)) {
            return false;
        }

        // 
        vec3 inv_dir = vec3(1.0 / r.direction().x(), 1.0 / r.direction().y(), 1.0 / r.direction().z());

        vec3 t_min = (vec3(bbox.x.min, bbox.y.min, bbox.z.min) - r.origin()) * inv_dir;
        vec3 t_max = (vec3(bbox.x.max, bbox.y.max, bbox.z.max) - r.origin()) * inv_dir;


        double t_entry = std::max(std::max(std::min(t_min.x(), t_max.x()), std::min(t_min.y(), t_max.y())), std::min(t_min.z(), t_max.z()));
        double t_exit = std::min(std::min(std::max(t_min.x(), t_max.x()), std::max(t_min.y(), t_max.y())), std::max(t_min.z(), t_max.z()));

        if (t_entry > t_exit || t_exit < ray_t.min) {
            return false;
        }

        t_entry = std::max(t_entry, ray_t.min);

        vec3 entry_point = r.at(t_entry);

        vec3 grid_coord = vec3(
            (entry_point.x() - bbox.x.min) / cell_size.x(),
            (entry_point.y() - bbox.y.min) / cell_size.y(),
            (entry_point.z() - bbox.z.min) / cell_size.z()
        );

        int x = static_cast<int>(grid_coord.x());
        int y = static_cast<int>(grid_coord.y());
        int z = static_cast<int>(grid_coord.z());

        
        vec3 delta_t = vec3(std::abs(cell_size.x() * inv_dir.x()), std::abs(cell_size.y() * inv_dir.y()), std::abs(cell_size.z() * inv_dir.z()));
        int step_x = (r.direction().x() >= 0) ? 1 : -1;
        int step_y = (r.direction().y() >= 0) ? 1 : -1;
        int step_z = (r.direction().z() >= 0) ? 1 : -1;

        double max_x = (r.direction().x() >= 0) ? ((x + 1) * cell_size.x() + bbox.x.min - entry_point.x()) * inv_dir.x() : (x * cell_size.x() + bbox.x.min - entry_point.x()) * inv_dir.x();
        double max_y = (r.direction().y() >= 0) ? ((y + 1) * cell_size.y() + bbox.y.min - entry_point.y()) * inv_dir.y() : (y * cell_size.y() + bbox.y.min - entry_point.y()) * inv_dir.y();
        double max_z = (r.direction().z() >= 0) ? ((z + 1) * cell_size.z() + bbox.z.min - entry_point.z()) * inv_dir.z() : (z * cell_size.z() + bbox.z.min - entry_point.z()) * inv_dir.z();

        bool hit_anything = false;
        while (t_entry < t_exit) {
            int index = x + y * cells_per_axis + z * cells_per_axis * cells_per_axis;
            if (index >= 0 && index < cells.size()) {
                for (const auto& object : cells[index]) {
                    if (object->hit(r, ray_t, rec, state)) {
                        hit_anything = true;
                        ray_t.max = rec.t;
                        t_exit = ray_t.max;
                    }
                }
            }

            if (max_x < max_y) {
                if (max_x < max_z) {
                    x += step_x;
                    if (x < 0 || x >= cells_per_axis) break;
                    t_entry += max_x;
                    max_x = delta_t.x();
                }
                else {
                    z += step_z;
                    if (z < 0 || z >= cells_per_axis) break;
                    t_entry += max_z;
                    max_z = delta_t.z();
                }
            }
            else {
                if (max_y < max_z) {
                    y += step_y;
                    if (y < 0 || y >= cells_per_axis) break;
                    t_entry += max_y;
                    max_y = delta_t.y();
                }
                else {
                    z += step_z;
                    if (z < 0 || z >= cells_per_axis) break;
                    t_entry += max_z;
                    max_z = delta_t.z();
                }
            }
        }




        return hit_anything;
    }

    aabb bounding_box() const override {
        return bbox;
    }

private:
    aabb bbox;
    int cells_per_axis;
    vec3 cell_size;
    std::vector<std::vector<shared_ptr<hittable>>> cells;
};


#endif