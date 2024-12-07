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


#endif
