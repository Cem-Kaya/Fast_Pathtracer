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

#include "rtweekend.h"

#include "bvh.h"
#include "camera.h"
#include "constant_medium.h"
#include "hittable.h"
#include "hittable_list.h"
#include "material.h"
#include "quad.h"
#include "sphere.h"
#include "texture.h"
#include "triangle.h"

#include <chrono>
#include <fstream>


void bouncing_spheres() {
	hittable_list world;

	auto checker = make_shared<checker_texture>(0.32, color(.2, .3, .1), color(.9, .9, .9));
	world.add(make_shared<sphere>(point3(0,-1000,0), 1000, make_shared<lambertian>(checker)));

	for (int a = -11; a < 11; a++) {
		for (int b = -11; b < 11; b++) {
			auto choose_mat = random_double();
			point3 center(a + 0.9*random_double(), 0.2, b + 0.9*random_double());

			if ((center - point3(4, 0.2, 0)).length() > 0.9) {
				shared_ptr<material> sphere_material;

				if (choose_mat < 0.8) {
					// diffuse
					auto albedo = color::random() * color::random();
					sphere_material = make_shared<lambertian>(albedo);
					auto center2 = center + vec3(0, random_double(0,.5), 0);
					world.add(make_shared<sphere>(center, center2, 0.2, sphere_material));
				} else if (choose_mat < 0.95) {
					// metal
					auto albedo = color::random(0.5, 1);
					auto fuzz = random_double(0, 0.5);
					sphere_material = make_shared<metal>(albedo, fuzz);
					world.add(make_shared<sphere>(center, 0.2, sphere_material));
				} else {
					// glass
					sphere_material = make_shared<dielectric>(1.5);
					world.add(make_shared<sphere>(center, 0.2, sphere_material));
				}
			}
		}
	}

	auto material1 = make_shared<dielectric>(1.5);
	world.add(make_shared<sphere>(point3(0, 1, 0), 1.0, material1));

	auto material2 = make_shared<lambertian>(color(0.4, 0.2, 0.1));
	world.add(make_shared<sphere>(point3(-4, 1, 0), 1.0, material2));

	auto material3 = make_shared<metal>(color(0.7, 0.6, 0.5), 0.0);
	world.add(make_shared<sphere>(point3(4, 1, 0), 1.0, material3));

	world = hittable_list(make_shared<bvh_node>(world));

	camera cam;

	cam.aspect_ratio      = 16.0 / 9.0;
	cam.image_width       = 400;
	cam.samples_per_pixel = 100;
	cam.max_depth         = 50;
	cam.background        = color(0.70, 0.80, 1.00);

	cam.vfov     = 20;
	cam.lookfrom = point3(13,2,3);
	cam.lookat   = point3(0,0,0);
	cam.vup      = vec3(0,1,0);

	cam.defocus_angle = 0.6;
	cam.focus_dist    = 10.0;

	cam.render(world);
}


void checkered_spheres() {
	hittable_list world;

	auto checker = make_shared<checker_texture>(0.32, color(.2, .3, .1), color(.9, .9, .9));

	world.add(make_shared<sphere>(point3(0,-10, 0), 10, make_shared<lambertian>(checker)));
	world.add(make_shared<sphere>(point3(0, 10, 0), 10, make_shared<lambertian>(checker)));

	camera cam;

	cam.aspect_ratio      = 16.0 / 9.0;
	cam.image_width       = 400;
	cam.samples_per_pixel = 100;
	cam.max_depth         = 50;
	cam.background        = color(0.70, 0.80, 1.00);

	cam.vfov     = 20;
	cam.lookfrom = point3(13,2,3);
	cam.lookat   = point3(0,0,0);
	cam.vup      = vec3(0,1,0);

	cam.defocus_angle = 0;

	cam.render(world);
}


void earth() {
	auto earth_texture = make_shared<image_texture>("earthmap.jpg");
	auto earth_surface = make_shared<lambertian>(earth_texture);
	auto globe = make_shared<sphere>(point3(0,0,0), 2, earth_surface);

	camera cam;

	cam.aspect_ratio      = 16.0 / 9.0;
	cam.image_width       = 400;
	cam.samples_per_pixel = 100;
	cam.max_depth         = 50;
	cam.background        = color(0.70, 0.80, 1.00);

	cam.vfov     = 20;
	cam.lookfrom = point3(0,0,12);
	cam.lookat   = point3(0,0,0);
	cam.vup      = vec3(0,1,0);

	cam.defocus_angle = 0;

	cam.render(hittable_list(globe));
}


void perlin_spheres() {
	hittable_list world;

	auto pertext = make_shared<noise_texture>(4);
	world.add(make_shared<sphere>(point3(0,-1000,0), 1000, make_shared<lambertian>(pertext)));
	world.add(make_shared<sphere>(point3(0,2,0), 2, make_shared<lambertian>(pertext)));

	camera cam;

	cam.aspect_ratio      = 16.0 / 9.0;
	cam.image_width       = 400;
	cam.samples_per_pixel = 100;
	cam.max_depth         = 50;
	cam.background        = color(0.70, 0.80, 1.00);

	cam.vfov     = 20;
	cam.lookfrom = point3(13,2,3);
	cam.lookat   = point3(0,0,0);
	cam.vup      = vec3(0,1,0);

	cam.defocus_angle = 0;

	cam.render(world);
}


void quads() {
	hittable_list world;

	// Materials
	auto left_red     = make_shared<lambertian>(color(1.0, 0.2, 0.2));
	auto back_green   = make_shared<lambertian>(color(0.2, 1.0, 0.2));
	auto right_blue   = make_shared<lambertian>(color(0.2, 0.2, 1.0));
	auto upper_orange = make_shared<lambertian>(color(1.0, 0.5, 0.0));
	auto lower_teal   = make_shared<lambertian>(color(0.2, 0.8, 0.8));

	// Quads
	world.add(make_shared<quad>(point3(-3,-2, 5), vec3(0, 0,-4), vec3(0, 4, 0), left_red));
	world.add(make_shared<quad>(point3(-2,-2, 0), vec3(4, 0, 0), vec3(0, 4, 0), back_green));
	world.add(make_shared<quad>(point3( 3,-2, 1), vec3(0, 0, 4), vec3(0, 4, 0), right_blue));
	world.add(make_shared<quad>(point3(-2, 3, 1), vec3(4, 0, 0), vec3(0, 0, 4), upper_orange));
	world.add(make_shared<quad>(point3(-2,-3, 5), vec3(4, 0, 0), vec3(0, 0,-4), lower_teal));

	camera cam;

	cam.aspect_ratio      = 1.0;
	cam.image_width       = 400;
	cam.samples_per_pixel = 100;
	cam.max_depth         = 50;
	cam.background        = color(0.70, 0.80, 1.00);

	cam.vfov     = 80;
	cam.lookfrom = point3(0,0,9);
	cam.lookat   = point3(0,0,0);
	cam.vup      = vec3(0,1,0);

	cam.defocus_angle = 0;

	cam.render(world);
}


void simple_light() {
	hittable_list world;

	auto pertext = make_shared<noise_texture>(4);
	world.add(make_shared<sphere>(point3(0,-1000,0), 1000, make_shared<lambertian>(pertext)));
	world.add(make_shared<sphere>(point3(0,2,0), 2, make_shared<lambertian>(pertext)));

	auto difflight = make_shared<diffuse_light>(color(4,4,4));
	world.add(make_shared<sphere>(point3(0,7,0), 2, difflight));
	world.add(make_shared<quad>(point3(3,1,-2), vec3(2,0,0), vec3(0,2,0), difflight));

	camera cam;

	cam.aspect_ratio      = 16.0 / 9.0;
	cam.image_width       = 400;
	cam.samples_per_pixel = 100;
	cam.max_depth         = 50;
	cam.background        = color(0,0,0);

	cam.vfov     = 20;
	cam.lookfrom = point3(26,3,6);
	cam.lookat   = point3(0,2,0);
	cam.vup      = vec3(0,1,0);

	cam.defocus_angle = 0;

	cam.render(world);
}


void cornell_box() {
	hittable_list world;

	auto red   = make_shared<lambertian>(color(.65, .05, .05));
	auto white = make_shared<lambertian>(color(.73, .73, .73));
	auto green = make_shared<lambertian>(color(.12, .45, .15));
	auto light = make_shared<diffuse_light>(color(15, 15, 15));

	world.add(make_shared<quad>(point3(555,0,0), vec3(0,555,0), vec3(0,0,555), green));
	world.add(make_shared<quad>(point3(0,0,0), vec3(0,555,0), vec3(0,0,555), red));
	world.add(make_shared<quad>(point3(343, 554, 332), vec3(-130,0,0), vec3(0,0,-105), light));
	world.add(make_shared<quad>(point3(0,0,0), vec3(555,0,0), vec3(0,0,555), white));
	world.add(make_shared<quad>(point3(555,555,555), vec3(-555,0,0), vec3(0,0,-555), white));
	world.add(make_shared<quad>(point3(0,0,555), vec3(555,0,0), vec3(0,555,0), white));

	shared_ptr<hittable> box1 = box(point3(0,0,0), point3(165,330,165), white);
	box1 = make_shared<rotate_y>(box1, 15);
	box1 = make_shared<translate>(box1, vec3(265,0,295));
	world.add(box1);

	shared_ptr<hittable> box2 = box(point3(0,0,0), point3(165,165,165), white);
	box2 = make_shared<rotate_y>(box2, -18);
	box2 = make_shared<translate>(box2, vec3(130,0,65));
	world.add(box2);

	camera cam;

	cam.aspect_ratio      = 1.0;
	cam.image_width       = 600;
	cam.samples_per_pixel = 200;
	cam.max_depth         = 50;
	cam.background        = color(0,0,0);

	cam.vfov     = 40;
	cam.lookfrom = point3(278, 278, -800);
	cam.lookat   = point3(278, 278, 0);
	cam.vup      = vec3(0,1,0);

	cam.defocus_angle = 0;

	cam.render(world);
}


void cornell_smoke() {
	hittable_list world;

	auto red   = make_shared<lambertian>(color(.65, .05, .05));
	auto white = make_shared<lambertian>(color(.73, .73, .73));
	auto green = make_shared<lambertian>(color(.12, .45, .15));
	auto light = make_shared<diffuse_light>(color(7, 7, 7));

	world.add(make_shared<quad>(point3(555,0,0), vec3(0,555,0), vec3(0,0,555), green));
	world.add(make_shared<quad>(point3(0,0,0), vec3(0,555,0), vec3(0,0,555), red));
	world.add(make_shared<quad>(point3(113,554,127), vec3(330,0,0), vec3(0,0,305), light));
	world.add(make_shared<quad>(point3(0,555,0), vec3(555,0,0), vec3(0,0,555), white));
	world.add(make_shared<quad>(point3(0,0,0), vec3(555,0,0), vec3(0,0,555), white));
	world.add(make_shared<quad>(point3(0,0,555), vec3(555,0,0), vec3(0,555,0), white));

	shared_ptr<hittable> box1 = box(point3(0,0,0), point3(165,330,165), white);
	box1 = make_shared<rotate_y>(box1, 15);
	box1 = make_shared<translate>(box1, vec3(265,0,295));

	shared_ptr<hittable> box2 = box(point3(0,0,0), point3(165,165,165), white);
	box2 = make_shared<rotate_y>(box2, -18);
	box2 = make_shared<translate>(box2, vec3(130,0,65));

	world.add(make_shared<constant_medium>(box1, 0.01, color(0,0,0)));
	world.add(make_shared<constant_medium>(box2, 0.01, color(1,1,1)));

	camera cam;

	cam.aspect_ratio      = 1.0;
	cam.image_width       = 600;
	cam.samples_per_pixel = 200;
	cam.max_depth         = 50;
	cam.background        = color(0,0,0);

	cam.vfov     = 40;
	cam.lookfrom = point3(278, 278, -800);
	cam.lookat   = point3(278, 278, 0);
	cam.vup      = vec3(0,1,0);

	cam.defocus_angle = 0;

	cam.render(world);
}


void final_scene(int image_width, int samples_per_pixel, int max_depth) {
	hittable_list boxes1;
	auto ground = make_shared<lambertian>(color(0.48, 0.83, 0.53));

	int boxes_per_side = 20;
	for (int i = 0; i < boxes_per_side; i++) {
		for (int j = 0; j < boxes_per_side; j++) {
			auto w = 100.0;
			auto x0 = -1000.0 + i*w;
			auto z0 = -1000.0 + j*w;
			auto y0 = 0.0;
			auto x1 = x0 + w;
			auto y1 = random_double(1,101);
			auto z1 = z0 + w;

			boxes1.add(box(point3(x0,y0,z0), point3(x1,y1,z1), ground));
		}
	}

	hittable_list world;

	world.add(make_shared<bvh_node>(boxes1));

	auto light = make_shared<diffuse_light>(color(7, 7, 7));
	world.add(make_shared<quad>(point3(123,554,147), vec3(300,0,0), vec3(0,0,265), light));

	auto center1 = point3(400, 400, 200);
	auto center2 = center1 + vec3(30,0,0);
	auto sphere_material = make_shared<lambertian>(color(0.7, 0.3, 0.1));
	world.add(make_shared<sphere>(center1, center2, 50, sphere_material));

	world.add(make_shared<sphere>(point3(260, 150, 45), 50, make_shared<dielectric>(1.5)));
	world.add(make_shared<sphere>(
		point3(0, 150, 145), 50, make_shared<metal>(color(0.8, 0.8, 0.9), 1.0)
	));

	auto boundary = make_shared<sphere>(point3(360,150,145), 70, make_shared<dielectric>(1.5));
	world.add(boundary);
	world.add(make_shared<constant_medium>(boundary, 0.2, color(0.2, 0.4, 0.9)));
	boundary = make_shared<sphere>(point3(0,0,0), 5000, make_shared<dielectric>(1.5));
	world.add(make_shared<constant_medium>(boundary, .0001, color(1,1,1)));

	auto emat = make_shared<lambertian>(make_shared<image_texture>("earthmap.jpg"));
	world.add(make_shared<sphere>(point3(400,200,400), 100, emat));
	auto pertext = make_shared<noise_texture>(0.2);
	world.add(make_shared<sphere>(point3(220,280,300), 80, make_shared<lambertian>(pertext)));

	hittable_list boxes2;
	auto white = make_shared<lambertian>(color(.73, .73, .73));
	int ns = 1000;
	for (int j = 0; j < ns; j++) {
		boxes2.add(make_shared<sphere>(point3::random(0,165), 10, white));
	}

	world.add(make_shared<translate>(
		make_shared<rotate_y>(
			make_shared<bvh_node>(boxes2), 15),
			vec3(-100,270,395)
		)
	);

	camera cam;

	cam.aspect_ratio      = 1.0;
	cam.image_width       = image_width;
	cam.samples_per_pixel = samples_per_pixel;
	cam.max_depth         = max_depth;
	cam.background        = color(0,0,0);

	cam.vfov     = 40;
	cam.lookfrom = point3(478, 278, -600);
	cam.lookat   = point3(278, 278, 0);
	cam.vup      = vec3(0,1,0);

	cam.defocus_angle = 0;

	cam.render(world);
}


void mesh_scene( std::string model_name) {
	auto mat = make_shared<lambertian>(color(0.8, 0.3, 0.3));
	auto mesh = parse_obj(model_name, mat);

	hittable_list world;
	world.add(mesh);

	// BVH ON   :D 	
	auto start_time = std::chrono::high_resolution_clock::now(); // Start the timer
	//world = hittable_list(make_shared<bvh_node>(world));
	//world = hittable_list(make_shared<octree_node>(world));
	world = hittable_list(make_shared<grid_acceleration>(world));

	auto end_time = std::chrono::high_resolution_clock::now(); // End the timer
	auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
	std::cout << "BVH construction time: " << elapsed_time << " ms" << std::endl;

	//////////////////////////////////

	camera cam;

	cam.aspect_ratio = 16.0 / 9.0;
	cam.image_width = 1080;
	cam.samples_per_pixel = 100;
	cam.max_depth = 50 ;
	cam.background = color(0.70, 0.80, 1.00);

	cam.vfov = 30; 
	cam.lookfrom = point3(5,10,100);
	cam.lookat = point3(0, 0, 0);
	cam.vup = vec3(0, 1, 0);

	cam.defocus_angle = 0;
	cam.render(world);
}


void d20_scene() {
	hittable_list world;

	// Ground with checkerboard texture
	auto checker = make_shared<checker_texture>(0.32, color(0.2, 0.3, 0.1), color(0.9, 0.9, 0.9));
	auto ground_material = make_shared<lambertian>(checker);
	world.add(make_shared<sphere>(point3(0, -1000.5, 0), 1000, ground_material));

	// Load D20 die
	auto d20_material = make_shared<lambertian>(color(0.8, 0.3, 0.3)); // Red material
	auto d20_die = parse_obj("../../../mesh/d20.obj", d20_material);
	world.add(d20_die);

	// Area light: a quad above the die
	auto light_material = make_shared<diffuse_light>(color(10, 10, 10));
	auto light_quad = make_shared<quad>(
		point3(-2, 5, -2),         // Bottom-left corner
		vec3(4, 0, 0),             // Width vector
		vec3(0, 0, 4),             // Height vector
		light_material
	);
	world.add(light_quad);

	// Additional visible light source: a bright sphere
	auto sphere_light_material = make_shared<diffuse_light>(color(20, 20, 20));
	world.add(make_shared<sphere>(point3(2, 6, -2), 1, sphere_light_material)); // Visible light sphere


	// BVH ON   :D 
	auto start_time = std::chrono::high_resolution_clock::now(); // Start the timer
	world = hittable_list(make_shared<bvh_node>(world));
	auto end_time = std::chrono::high_resolution_clock::now(); // End the timer
	auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
	std::cout << "BVH construction time: " << elapsed_time << " ms" << std::endl;
	//////////////////////////////////////



	// Camera setup
	camera cam;
	cam.aspect_ratio = 16.0 / 9.0;
	cam.image_width = 400;              // Higher resolution for better quality
	cam.samples_per_pixel = 50;         // More samples for better anti-aliasing
	cam.max_depth = 20;                 // Maximum depth for recursion
	cam.background = color(0.02, 0.02, 0.075); //  background

	cam.vfov = 20.0;
	cam.lookfrom = point3(3, 2, 6);  // Position of the camera
	cam.lookat = point3(0, 0, 0);    // Where the camera looks at
	cam.vup = vec3(0, 1, 0);         // Up direction

	// Render the scene
	cam.render(world);
}


void fancy_mesh_scene(const std::string& model_name) {
	// Load the mesh and create materials for the two copies
	auto red_mesh_mat = make_shared<lambertian>(color(0.8, 0.3, 0.3)); // Red mesh
	auto green_mesh_mat = make_shared<lambertian>(color(0.2, 0.8, 0.2)); // Green mesh
	auto red_mesh = parse_obj(model_name, red_mesh_mat);
	auto green_mesh = parse_obj(model_name, green_mesh_mat);

	hittable_list world;

	// Ground: a large checker-textured quad
	auto checker = make_shared<checker_texture>(0.5, color(0.2, 0.2, 0.2), color(0.9, 0.9, 0.9));
	auto ground_material = make_shared<lambertian>(checker);
	world.add(make_shared<quad>(
		point3(-1000, 0, -1000),
		vec3(2000, 0, 0),
		vec3(0, 0, 2000),
		ground_material
	));

	// First copy of the mesh (red), centered and raised
	auto mesh_center_correction = vec3(-(14.41 + 27.07) / 2.0, -0.7, -(8.02 + 7.46) / 2.0);
	auto first_mesh = make_shared<translate>(red_mesh, vec3(10, 0, 0) + mesh_center_correction);
	world.add(first_mesh);

	// Second copy of the mesh (green), shifted right relative to the first
	auto second_mesh = make_shared<translate>(green_mesh, vec3(30, 0, 0) + mesh_center_correction);
	world.add(second_mesh);

	// Mirror behind the scene: a large, perfectly reflective quad
	auto mirror_material = make_shared<metal>(color(0.95, 0.95, 0.95), 0.0);
	world.add(make_shared<quad>(
		point3(-1000, -1, -30),
		vec3(2000, 0, 0),
		vec3(0, 2000, 0),
		mirror_material
	));

	// Back wall (white)
	auto back_wall_material = make_shared<lambertian>(color(0.9, 0.9, 0.9));
	world.add(make_shared<quad>(
		point3(-1000, 0, -100),
		vec3(2000, 0, 0),
		vec3(0, 2000, 0),
		back_wall_material
	));

	// Rainbow colored lights, shifted right for better alignment
	double light_y = 15.0; // Height of the lights
	double light_z = 15.0; // Z-axis position of the lights
	double quad_size = 3.0;

	auto red_light = make_shared<diffuse_light>(color(1, 0, 0));
	auto orange_light = make_shared<diffuse_light>(color(1, 0.5, 0));
	auto yellow_light = make_shared<diffuse_light>(color(1, 1, 0));
	auto green_light = make_shared<diffuse_light>(color(0, 1, 0));
	auto blue_light = make_shared<diffuse_light>(color(0, 0, 1));
	auto indigo_light = make_shared<diffuse_light>(color(0.29, 0, 0.51));
	auto violet_light = make_shared<diffuse_light>(color(0.56, 0, 1));

	// Position rainbow lights shifted right
	world.add(make_shared<quad>(point3(0, light_y, light_z), vec3(quad_size, 0, 0), vec3(0, quad_size, 0), red_light));
	world.add(make_shared<quad>(point3(5, light_y, light_z), vec3(quad_size, 0, 0), vec3(0, quad_size, 0), orange_light));
	world.add(make_shared<quad>(point3(10, light_y, light_z), vec3(quad_size, 0, 0), vec3(0, quad_size, 0), yellow_light));
	world.add(make_shared<quad>(point3(15, light_y, light_z), vec3(quad_size, 0, 0), vec3(0, quad_size, 0), green_light));
	world.add(make_shared<quad>(point3(20, light_y, light_z), vec3(quad_size, 0, 0), vec3(0, quad_size, 0), blue_light));
	world.add(make_shared<quad>(point3(25, light_y, light_z), vec3(quad_size, 0, 0), vec3(0, quad_size, 0), indigo_light));
	world.add(make_shared<quad>(point3(30, light_y, light_z), vec3(quad_size, 0, 0), vec3(0, quad_size, 0), violet_light));

	// Glass spheres
	auto glass = make_shared<dielectric>(1.5);
	world.add(make_shared<sphere>(point3(5, 1, -5), 1.0, glass));
	world.add(make_shared<sphere>(point3(10, 1, 0), 1.0, glass));
	world.add(make_shared<sphere>(point3(25, 1, 0), 1.0, glass));
	world.add(make_shared<sphere>(point3(30, 1, 5), 1.0, glass));

	// Glass sphere close to the camera
	world.add(make_shared<sphere>(point3(18, 2, 20), 2.0, glass));
	// fancy mesh scene ! ! !
	// Construct BVH
	auto start_time = std::chrono::high_resolution_clock::now();
	world = hittable_list(make_shared<bvh_node>(world));
	//world = hittable_list(make_shared<octree_node>(world));
	//world = hittable_list(make_shared<grid_acceleration>(world));


	auto end_time = std::chrono::high_resolution_clock::now();
	auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
	std::cout << "Accalaration structure construction time: " << elapsed_time << " micro seconds " << std::endl;

	// Camera setup
	camera cam;
	cam.aspect_ratio = 16.0 / 9.0;
	cam.image_width = 640;
	cam.samples_per_pixel = 64;
	cam.max_depth = 20;
	cam.background = color(0.70, 0.80, 1.00);
	// c1 
	cam.vfov = 30;
	cam.lookfrom = point3(20, 2, 30);
	cam.lookat = point3(20, 5, 0);
	cam.vup = vec3(0, 1, 0);
	//c2 45 degree to the right 
	cam.lookfrom = point3(20, 2, 30);
	cam.lookat = point3(41.21, 5, 8.79); // cos 
	cam.vup = vec3(0, 1, 0);


	cam.defocus_angle = 0;

	// Render the scene
	cam.render(world);
}


void shadow_scene(std::string model_name) {
	auto mat = make_shared<lambertian>(color(0.3, 0.8, 0.3));
	auto mesh = parse_obj(model_name, mat);
	auto mesh_correction = vec3(-10.091, -10.732, -0.106); // Centering the mesh
	hittable_list world;
	world.add(make_shared<translate>(mesh, mesh_correction));


	world.add(make_shared<quad>(
		point3(-1000, -0.5, -1000),
		vec3(2000, 0, 0),
		vec3(0, 0, 2000),
		make_shared<lambertian>(color(0.8, 0.8, 0.8))
	));

	// Add more light sources
	auto bright_light = make_shared<diffuse_light>(color(1, 0.75, 0.50)); //  intensity
	world.add(make_shared<quad>(point3(-4, 5, 0), vec3(3, 0, 0), vec3(0, 3, 0), bright_light));
	auto bright_light2 = make_shared<diffuse_light>(color(0.5, 0.75, 1)); //  intensity
	world.add(make_shared<sphere>(point3(3, 10, 3), 1.0, bright_light2)); // Overhead light
	// dark scene 
	// Construct BVH
	auto start_time = std::chrono::high_resolution_clock::now();
	//world = hittable_list(make_shared<bvh_node>(world));
	//world = hittable_list(make_shared<octree_node>(world));
	world = hittable_list(make_shared<grid_acceleration>(world));


	auto end_time = std::chrono::high_resolution_clock::now();
	auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
	std::cout << "Accalaration structure construction time: " << elapsed_time << " micro seconds " << std::endl;



	// Camera setup for better visibility
	camera cam;
	cam.aspect_ratio = 16.0 / 9.0;
	cam.image_width = 640;
	cam.samples_per_pixel = 64; // Improved quality
	cam.max_depth = 20;



	cam.background = color(0.01, 0.01, 0.02); // 
	cam.vfov = 22.5;
	cam.lookfrom = point3(10, 25, 25); // Adjusted to focus on the mesh
	cam.lookat = point3(0, 0, 0); // Looking at the centroid
	cam.vup = vec3(0, 1, 0);
	cam.defocus_angle = 0.1;
	cam.focus_dist = 20.0;

	cam.render(world);
}



void Trex(const std::string& model_name) {
	// Load the mesh and assign it a basic material
	auto mesh_material = make_shared<lambertian>(color(0.6, 0.6, 0.6)); //  material
	auto mesh = parse_obj(model_name, mesh_material);

	hittable_list world;

	// Add the mesh to the world
	world.add(mesh);

	// Set up a green background
	auto green_background = make_shared<diffuse_light>(color(0.0, 1.0, 0.0)); // Bright green
	world.add(make_shared<quad>(
		point3(-1000, -1000, -1000),  // Bottom-left corner
		vec3(2000, 0, 0),            // Width vector
		vec3(0, 2000, 0),            // Height vector
		green_background
	));
	//trex 
	// fancy mesh scene ! ! !
	// Construct BVH
	auto start_time = std::chrono::high_resolution_clock::now();
	//world = hittable_list(make_shared<bvh_node>(world));
	//world = hittable_list(make_shared<octree_node>(world));
	world = hittable_list(make_shared<grid_acceleration>(world));


	auto end_time = std::chrono::high_resolution_clock::now();
	auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
	std::cout << "Accalaration structure construction time: " << elapsed_time << " micro seconds " << std::endl;



	// Camera setup
	camera cam;
	cam.aspect_ratio = 16.0 / 9.0;
	cam.image_width = 640; // 640 
	cam.samples_per_pixel = 4;
	cam.max_depth = 20;
	cam.background = color(0.4, 0.6, 0.4); // Set green background color

	// Position the camera at the center of the mesh's AABB
	
	cam.lookfrom = point3(0.2359, -1.9712, 0.6302);  // 
	cam.lookat = point3(0.0061, -0.0003, -0.3);  // 



	cam.vfov = 10 ;                                 // Field of view

	// Render the scene
	cam.render(world);
}


int main() {
	switch (12) {
		case 1:  bouncing_spheres();          break;
		case 2:  checkered_spheres();         break;
		case 3:  earth();                     break;
		case 4:  perlin_spheres();            break;
		case 5:  quads();                     break;
		case 6:  simple_light();              break;
		case 7:  cornell_box();               break;
		case 8:  cornell_smoke();             break;
		case 9:  final_scene(800, 10000, 40); break;
		case 10: mesh_scene("../../../mesh/model_to_big.obj");    break;
		case 11: mesh_scene("../../../mesh/model9.obj"); break;
		case 12: fancy_mesh_scene("../../../mesh/model9.obj");	 ; break;
		case 13: shadow_scene("../../../mesh/separated_mesh_5.obj"); break;
		case 14: Trex("../../../mesh/mesh_from_nerf.obj"); break;
		case 20: d20_scene();                 break;
		default: final_scene(400,   250,  4); break;
	}
}
