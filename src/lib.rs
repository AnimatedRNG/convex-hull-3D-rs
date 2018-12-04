extern crate nalgebra_glm as glm;

use std::collections::{HashSet, VecDeque};
use std::f32::consts::PI;

type PointCloud = Vec<glm::Vec3>;

pub fn convex_hull(points: &PointCloud) -> PointCloud {
    let mut new_mesh = HashSet::new();

    let mut leftmost = (0, glm::vec3(1e10, 0.0, 0.0));
    for (index, pt) in points.iter().enumerate() {
        if pt.x < leftmost.1.x {
            leftmost = (index, *pt);
        }
    }

    let v0 = glm::vec3(1.0, 0.0, 0.0);
    let mut most_convex = (0, -1e10);
    for (index, pt) in points.iter().enumerate() {
        let diff = (pt - leftmost.1).normalize();
        let angle = diff.dot(&v0).acos();
        if angle > most_convex.1 && angle < PI {
            most_convex = (index, angle)
        }
    }
    new_mesh.insert(leftmost.0);
    new_mesh.insert(most_convex.0);

    let mut visited: HashSet<(usize, usize)> = HashSet::new();
    let mut agenda: VecDeque<(usize, usize)> = VecDeque::new();
    agenda.push_back((leftmost.0, most_convex.0));

    while agenda.len() > 0 {
        let e = agenda.pop_front().unwrap();

        if visited.contains(&e) {
            continue;
        }
        println!("Current edge is {:?}", e);

        let mut most_convex = (0, -1e10);
        for (index, pt) in points.iter().enumerate() {
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
        new_mesh.insert(most_convex.0);
        agenda.push_back((e.1, most_convex.0));
        agenda.push_back((most_convex.0, e.0));
        visited.insert(e.clone());
    }

    new_mesh.iter().map(|&ind| points[ind]).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tet() {
        let sphere: PointCloud = vec![
            glm::vec3(0.0, 0.0, 0.0),
            glm::vec3(1.0, 0.0, 0.0),
            glm::vec3(0.0, 1.0, 0.0),
            glm::vec3(0.0, 0.0, 1.0),
        ];

        assert!(convex_hull(&sphere).len() == 4)
    }

    #[test]
    fn test_sphere() {
        let sphere: PointCloud = (0..200)
            .map(|index| {
                let theta = ((index % 10) as f32 / 10.0) * (2.0 * PI);
                let phi = ((index % 100) as f32 / 100.0) * (2.0 * PI);
                let radius = (index as f32 / 200.0);
                let radius = 1.0;

                glm::vec3(
                    radius * phi.sin() * theta.cos(),
                    radius * phi.sin() * theta.sin(),
                    radius * phi.cos(),
                )
            })
            .collect();

        let sphere_hull = convex_hull(&sphere);
        assert!(sphere_hull.iter().map(|a| a.norm()).all(|a| a > 0.8));
    }
}
