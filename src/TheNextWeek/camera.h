#ifndef CAMERA_H
#define CAMERA_H
//==============================================================================================
// Originally written in 2016 by Peter Shirley <ptrshrl@gmail.com>
//
// To the extent possible under law, the author(s) have dedicated all copyright and related
// neighboring rights to this software to the public domain worldwide. This software is
// distributed without any warranty.
//
// You should have received a copy (see file COPYING.txt) of the CC0 Public Domain Dedication
// along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
//==============================================================================================

#include "hittable.h"
#include "material.h"
#include "bvh.h"

#include <sstream> 
#include <chrono>
#include <thread>
#include <fstream>
#include <cstdlib>
#include <ctime>

#include <thread>
#include <future>



class camera {
public:
    double aspect_ratio = 1.0;  // Ratio of image width over height
    int    image_width = 100;  // Rendered image width in pixel count
    int    samples_per_pixel = 10;   // Count of random samples for each pixel
    int    max_depth = 10;   // Maximum number of ray bounces into scene
    color  background;               // Scene background color

    double vfov = 90;              // Vertical view angle (field of view)
    point3 lookfrom = point3(0, 0, 0);   // Point camera is looking from
    point3 lookat = point3(0, 0, -1);  // Point camera is looking at
    vec3   vup = vec3(0, 1, 0);     // Camera-relative "up" direction

    double defocus_angle = 0;  // Variation angle of rays through each pixel
    double focus_dist = 10;    // Distance from camera lookfrom point to plane of perfect focus

    std::vector<pixel_efficiency_data> pixels;

    void camera::render(const hittable& world) {
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::system_clock::to_time_t(now);
        std::ostringstream oss;
        oss << timestamp;

        ppm_filename = oss.str() + ".ppm";
        metrics_filename = "metrics_" + oss.str() + ".csv";

        initialize_pixel_data();

        std::ofstream output_file(ppm_filename);
        if (!output_file.is_open()) {
            throw std::runtime_error("Failed to open output file: " + ppm_filename);
        }

        output_file << "P3\n" << image_width << ' ' << image_height << "\n255\n";

        // Number of threads to use
        int num_threads = std::thread::hardware_concurrency();
        if (num_threads < 1) num_threads = 4; // Fallback to 4 threads if detection fails

        // Divide the work by rows
        std::vector<std::future<void>> futures;
        int rows_per_thread = image_height / num_threads;

        // Shared scanline counter
        int remaining_scanlines = image_height;

        for (int t = 0; t < num_threads; t++) {
            int start_row = t * rows_per_thread;
            int end_row = (t == num_threads - 1) ? image_height : start_row + rows_per_thread;

            // Schedule each chunk to run asynchronously
            futures.push_back(std::async(std::launch::async, &camera::render_chunk, this, std::ref(world), start_row, end_row, std::ref(remaining_scanlines)));
        }

        // Wait for all threads to finish
        for (auto& f : futures) {
            f.get();
        }

        finalize_output(output_file);
    }

    void camera::render_chunk(const hittable& world, int start_row, int end_row, int& remaining_scanlines) {
        static std::mutex print_mutex; // Mutex to protect the print operation
        for (int j = start_row; j < end_row; j++) {
            for (int i = 0; i < image_width; i++) {
                pixel_efficiency_data& p = pixels[j * image_width + i];

                for (int s = 0; s < samples_per_pixel; s++) {
                    ray_state state;
                    ray r = get_ray(i, j);
                    color sample_color = ray_color(r, max_depth, world, state);

                    // Accumulate color
                    p.accumulated_color += sample_color;

                    // Accumulate metrics
                    p.total_traversal_steps += state.traversal_steps;
                    p.total_intersection_tests += state.intersection_tests;
                }
            }

            // Decrement the shared remaining scanlines counter
            {
                std::lock_guard<std::mutex> lock(print_mutex);
                remaining_scanlines--;
                std::clog << "\rScanlines remaining: " << remaining_scanlines << ' ' << std::flush;
            }
        }
    }


  
private:
    int    image_height;
    double pixel_samples_scale;
    point3 center;
    point3 pixel00_loc;
    vec3   pixel_delta_u;
    vec3   pixel_delta_v;
    vec3   u, v, w;
    vec3   defocus_disk_u;
    vec3   defocus_disk_v;

    std::string ppm_filename;
    std::string metrics_filename;

    void initialize_pixel_data() {
        image_height = int(image_width / aspect_ratio);
        if (image_height < 1) image_height = 1;

        pixel_samples_scale = 1.0 / samples_per_pixel;

        pixels.resize(image_width * image_height);
        for (auto& p : pixels) {
            p = pixel_efficiency_data();
        }

        initialize(); // initialize camera geometry
    }

    void finalize_output(std::ofstream& output_file) {
        // Optionally, open metrics file
        std::ofstream metrics_file(metrics_filename);
        metrics_file << "x,y,avg_traversal_steps,avg_intersection_tests,timestamp\n";

        for (int j = 0; j < image_height; j++) {
            for (int i = 0; i < image_width; i++) {
                const pixel_efficiency_data& p = pixels[j * image_width + i];

                // Compute the average color
                color final_color = pixel_samples_scale * p.accumulated_color;
                // Write final color in PPM
                write_color(output_file, final_color);

                // Compute average metrics per sample
                double avg_traversal = (double)p.total_traversal_steps / samples_per_pixel;
                double avg_intersect = (double)p.total_intersection_tests / samples_per_pixel;

                // Append timestamp to metrics as well
                metrics_file << i << "," << j << "," << avg_traversal << "," << avg_intersect << "\n";
            }
        }

        output_file.close();
        metrics_file.close();

        std::clog << "\nRender complete. Images and metrics written.\n";

        // Convert PPM to PNG
        std::string png_filename = ppm_filename.substr(0, ppm_filename.find_last_of('.')) + ".png";
        std::string convert_command = "magick convert " + ppm_filename + " " + png_filename;
        int convert_status = system(convert_command.c_str());
        if (convert_status != 0) {
            std::cerr << "Error: Failed to convert PPM to PNG." << std::endl;
        }

        // Open the PNG file (Windows)
        std::string open_command = "start " + png_filename;
        int open_status = system(open_command.c_str());
        if (open_status != 0) {
            std::cerr << "Error: Failed to open the PNG file." << std::endl;
        }
    }

    void initialize() {
        image_height = int(image_width / aspect_ratio);
        image_height = (image_height < 1) ? 1 : image_height;

        pixel_samples_scale = 1.0 / samples_per_pixel;

        center = lookfrom;

        // Determine viewport dimensions.
        auto theta = degrees_to_radians(vfov);
        auto h = std::tan(theta / 2);
        auto viewport_height = 2 * h * focus_dist;
        auto viewport_width = viewport_height * (double(image_width) / image_height);

        // Calculate the u,v,w unit basis vectors for the camera coordinate frame.
        w = unit_vector(lookfrom - lookat);
        u = unit_vector(cross(vup, w));
        v = cross(w, u);

        // Calculate the vectors across the horizontal and down the vertical viewport edges.
        vec3 viewport_u = viewport_width * u;    // Vector across viewport horizontal edge
        vec3 viewport_v = viewport_height * -v;  // Vector down viewport vertical edge

        // Calculate the horizontal and vertical delta vectors from pixel to pixel.
        pixel_delta_u = viewport_u / image_width;
        pixel_delta_v = viewport_v / image_height;

        // Calculate the location of the upper-left pixel.
        auto viewport_upper_left = center - (focus_dist * w) - viewport_u / 2 - viewport_v / 2;
        pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

        // Calculate the camera defocus disk basis vectors.
        auto defocus_radius = focus_dist * std::tan(degrees_to_radians(defocus_angle / 2));
        defocus_disk_u = u * defocus_radius;
        defocus_disk_v = v * defocus_radius;
    }

    ray get_ray(int i, int j) const {
        // Construct a camera ray originating from the defocus disk and directed at a randomly
        // sampled point around the pixel location i, j.

        auto offset = sample_square();
        auto pixel_sample = pixel00_loc
            + ((i + offset.x()) * pixel_delta_u)
            + ((j + offset.y()) * pixel_delta_v);

        auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
        auto ray_direction = pixel_sample - ray_origin;
        auto ray_time = random_double();

        return ray(ray_origin, ray_direction, ray_time);
    }

    vec3 sample_square() const {
        // Returns the vector to a random point in the [-.5,-.5]-[+.5,+.5] unit square.
        return vec3(random_double() - 0.5, random_double() - 0.5, 0);
    }

    point3 defocus_disk_sample() const {
        // Returns a random point in the camera defocus disk.
        auto p = random_in_unit_disk();
        return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
    }

    color ray_color(const ray& r, int depth, const hittable& world, ray_state& state) const {
        // If we've exceeded the ray bounce limit, no more light is gathered.
        if (depth <= 0)
            return color(0, 0, 0);

        hit_record rec;

        // Pass the state down into the world hit
        if (!world.hit(r, interval(0.001, infinity), rec, state))
            return background;

        ray scattered;
        color attenuation;
        color color_from_emission = rec.mat->emitted(rec.u, rec.v, rec.p);

        if (!rec.mat->scatter(r, rec, attenuation, scattered))
            return color_from_emission;

        color color_from_scatter = attenuation * ray_color(scattered, depth - 1, world, state);

        return color_from_emission + color_from_scatter;
    }

};

#endif
