extern crate nalgebra_glm as glm;

use std::collections::HashSet;
use std::f32::consts::PI;

type PointCloud = Vec<glm::Vec3>;

fn convex_hull(points: &PointCloud) -> PointCloud {
    let mut new_mesh = PointCloud::new();
    let mut agenda: HashSet<usize> = HashSet::new();
    for i in 0..points.len() {
        agenda.insert(i);
    }

    let mut leftmost = (0, glm::vec3(1e10, 0.0, 0.0));
    for (index, pt) in points.iter().enumerate() {
        if pt.x < leftmost.1.x {
            leftmost = (index, *pt);
        }
    }
    //agenda.remove(&leftmost.0);
    // Note that leftmost.0 is in the agenda

    let v0 = glm::vec3(1.0, 0.0, 0.0);
    let mut most_convex = (0, -1e10);
    for &index in &agenda {
        let pt = points[index];
        let diff = (pt - leftmost.1).normalize();
        let angle = diff.dot(&v0).acos();
        if angle > most_convex.1 && angle < PI {
            most_convex = (index, angle)
        }
    }
    new_mesh.push(leftmost.1);
    new_mesh.push(points[most_convex.0]);

    let mut e = (leftmost.0, most_convex.0);
    loop {
        println!("Current edge is {:?}", e);
        agenda.remove(&e.1);

        most_convex = (0, -1e10);
        for &index in &agenda {
            let pt = points[index];
            let diff = (points[e.1] - pt).normalize();
            let angle = diff.dot(&(points[e.1] - points[e.0]).normalize()).acos();
            if angle > most_convex.1 && angle < PI {
                most_convex = (index, angle)
            }
        }
        println!(
            "Picked point {} with angle {}",
            most_convex.0, most_convex.1
        );
        new_mesh.push(points[most_convex.0]);
        e = (e.1, most_convex.0);

        if most_convex.0 == leftmost.0 {
            break;
        }
    }

    new_mesh
}

fn main() {
    let tet: PointCloud = vec![
        glm::vec3(0.0, 0.0, 0.0),
        glm::vec3(1.0, 0.0, 0.0),
        glm::vec3(0.0, 1.0, 0.0),
        glm::vec3(0.0, 0.0, 1.0),
    ];

    let sphere: PointCloud = (0..100)
        .map(|index| {
            let theta = ((index % 10) as f32 / 10.0) * (2.0 * PI);
            let phi = ((index % 100) as f32 / 100.0) * (2.0 * PI);
            //let radius = (index as f32 / 1000.0);
            let radius = 1.0;

            glm::vec3(
                radius * phi.sin() * theta.cos(),
                radius * phi.sin() * theta.sin(),
                radius * phi.cos(),
            )
        })
        .collect();

    //println!("{:?}", convex_hull(&tet));
    let sphere_hull = convex_hull(&sphere);
    println!("Sphere: {:?}", &sphere);
    println!("Sphere hull: {:?}", sphere_hull);
    println!(
        "{:?}",
        convex_hull(&sphere)
            .iter()
            .map(|a| a.norm())
            .collect::<Vec<f32>>()
    );
}
