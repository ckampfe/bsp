//# https://en.wikipedia.org/wiki/Binary_space_partitioning

use nalgebra::{Point3, Vector3};

trait Polygon {
    fn points(&self) -> &[Point3<f32>];
    fn normal(&self) -> Vector3<f32>;
    fn test<P: Polygon>(&self, other: &P) -> Relation {
        self.split(other)
    }

    // TODO figure out how to make sure points are:
    // 1. ordered
    // 2. unique
    fn split<P: Polygon>(&self, plane: &P) -> Relation {
        let points = self.points();

        let [p0, p1, p2] = [points[0], points[1], points[2]];

        let plane_normal = plane.normal();

        let point_on_plane = plane.points()[0];

        let p0rel = plane_normal.dot(&(p0 - point_on_plane)).rel();
        let p1rel = plane_normal.dot(&(p1 - point_on_plane)).rel();
        let p2rel = plane_normal.dot(&(p2 - point_on_plane)).rel();

        let mut rels: [(Dot, Point3<f32>); 3] = [(p0rel, p0), (p1rel, p1), (p2rel, p2)];

        // sort the rels front to back
        rels.sort_unstable_by(|(rel1, _), (rel2, _)| rel1.partial_cmp(&rel2).unwrap());

        match rels {
            [(Dot::Coplanar, _p0), (Dot::Behind, _p1), (Dot::Behind, _p2)] => {
                // case where plane intersects one point, with two behind
                Relation::Behind // TODO is having this tri be behind correct?
            }
            [(Dot::Front, _p0), (Dot::Front, _p1), (Dot::Coplanar, _p2)] => {
                // case where plane intersects one point with two in front
                Relation::Front // TODO is having this tri be in front correct?
            }
            [(Dot::Coplanar, _p0), (Dot::Coplanar, _p1), (Dot::Behind, _p2)] => {
                // case where plane intersects two points, with one behind
                Relation::Behind // TODO is having this tri be behind correct?
            }
             [(Dot::Front, _p0), (Dot::Coplanar, _p1), (Dot::Coplanar, _p2)] => {
                // case where plane intersects two points, with one in front
                Relation::Front // TODO is having this tri be in front correct?
            }
            [(Dot::Front, p0), (Dot::Behind, p1), (Dot::Behind, p2)] => {
                // case where plane intersects no points, with one in front and two behind
                let mut front = Vec::with_capacity(3);
                let mut behind = Vec::with_capacity(4);

                let p0p1i = ray_to_plane_intersection(&(p0, p1), &plane.normal(), &plane.points()[0]);
                let p0p2i = ray_to_plane_intersection(&(p0, p2), &plane.normal(), &plane.points()[0]);

                front.push(p0);
                front.push(p0p1i);
                front.push(p0p2i);

                behind.push(p1);
                behind.push(p2);
                behind.push(p0p1i);
                behind.push(p0p2i);

                Relation::Split {
                    front, behind
                }
            }
            [(Dot::Front, p0), (Dot::Front, p1), (Dot::Behind, p2)] => {
                // case where plane intersects no points, with two in front and one behind
                let mut front = Vec::with_capacity(4);
                let mut behind = Vec::with_capacity(3);

                let p0p2i = ray_to_plane_intersection(&(p0, p2), &plane.normal(), &plane.points()[0]);
                let p1p2i = ray_to_plane_intersection(&(p1, p2), &plane.normal(), &plane.points()[0]);

                front.push(p0);
                front.push(p1);
                front.push(p0p2i);
                front.push(p1p2i);

                behind.push(p2);
                behind.push(p0p2i);
                behind.push(p1p2i);

                Relation::Split {
                    front, behind
                }
            }
            [(Dot::Front, p0), (Dot::Coplanar, p1), (Dot::Behind, p2)] => {
                // case where plane intersects one point, with one in front and one behind
                let mut front = Vec::with_capacity(3);
                let mut behind = Vec::with_capacity(3);

                let p0p2i= ray_to_plane_intersection(&(p0, p2), &plane.normal(), &plane.points()[0]);

                front.push(p0);
                front.push(p1);
                front.push(p0p2i);

                behind.push(p1);
                behind.push(p2);
                behind.push(p0p2i);

                Relation::Split {
                    front, behind
                }
            }
            [(Dot::Coplanar, _p0), (Dot::Coplanar, _p1), (Dot::Coplanar, _p2)] => {
                // case where all points are coplanar to the plane
                Relation::Coplanar
            }
            [(Dot::Front, _p0), (Dot::Front, _p1), (Dot::Front, _p2)] => {
                // case where all points are in front of plane
                Relation::Front
            }
            [(Dot::Behind, _p0), (Dot::Behind, _p1), (Dot::Behind, _p2)] => {
                // case where all points are behind plane
                Relation::Behind
            }
            _ => unreachable!("This should never happen as the points are sorted front to back by their dot product to the plane normal vector"),
        }
    }

    fn edges(&self) -> Vec<(Point3<f32>, Point3<f32>)> {
        let mut rot = self.points().to_owned();
        rot.rotate_left(1);
        self.points().iter().cloned().zip(rot).collect()
    }
}

trait ToDot: PartialEq {
    fn rel(self) -> Dot;
}

impl ToDot for f32 {
    fn rel(self) -> Dot {
        if self < 0.0 {
            Dot::Behind
        } else if self == 0.0 {
            Dot::Coplanar
        } else {
            Dot::Front
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
enum Dot {
    Front,
    Coplanar,
    Behind,
}

#[derive(Clone, Debug, PartialEq)]
enum Relation {
    Front,
    Behind,
    Coplanar,
    Split {
        front: Vec<Point3<f32>>,
        behind: Vec<Point3<f32>>,
    },
}

struct BSPTree<P> {
    points: Vec<P>,
}

impl<P> BSPTree<P>
where
    P: Polygon,
{
    fn new(polygons: Vec<P>) -> Self {
        Self::from(polygons.as_slice())
    }
}

impl<P> From<&[P]> for BSPTree<P>
where
    P: Polygon,
{
    fn from(polygons: &[P]) -> Self {
        todo!()
    }
}

impl<P> Iterator for BSPTree<P>
where
    P: Polygon,
{
    type Item = P;

    fn next(&mut self) -> Option<Self::Item> {
        todo!()
    }
}

// fn point_to_line_distance((p1, p2): (Point3<f32>, Point3<f32>), point: Point3<f32>) -> f32 {
//     let line_dist = nalgebra::distance_squared(&p2, &p1);
//
//     if line_dist == 0.0 {
//         return nalgebra::distance_squared(&p1, &point);
//     }
//
//     let line_segment = p2 - p1;
//
//     let point_to_min = point - p1;
//
//     let t = point_to_min.dot(&line_segment) / line_dist;
//     // let t = dot(point_to_min, line_segment) / line_dist;
//
//     nalgebra::distance(&point, &(p1 + (line_segment * t)))
// }
//
fn ray_to_plane_intersection(
    (p1, p2): &(Point3<f32>, Point3<f32>),
    plane_normal: &Vector3<f32>,
    plane_point: &Point3<f32>,
) -> Point3<f32> {
    let r = (plane_normal.dot(&(plane_point - p1))) / (plane_normal.dot(&(p2 - p1)));
    p1 + r * (p2 - p1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point3, Vector3};

    struct Triangle {
        normal: Vector3<f32>,
        points: Vec<Point3<f32>>,
    }

    impl Polygon for Triangle {
        fn normal(&self) -> Vector3<f32> {
            self.normal
        }
        fn points(&self) -> &[Point3<f32>] {
            self.points.as_slice()
        }
    }

    #[test]
    fn gets_edges_in_order() {
        let p0 = Point3::new(0.0, 0.0, 0.0);
        let p1 = Point3::new(0.0, 1.0, 0.0);
        let p2 = Point3::new(0.0, 0.0, 1.0);

        let triangle = Triangle {
            normal: Vector3::new(0., 0.0, 0.0),
            points: vec![p0, p1, p2],
        };

        let edges = triangle.edges();
        assert_eq!(edges.len(), 3);
        assert_eq!(edges, vec![(p0, p1), (p1, p2), (p2, p0)]);
    }

    #[test]
    fn splits_a_triangle_where_one_point_is_coplanar_one_is_in_front_and_one_behind() {
        let p0 = Point3::new(-1.0, 0.0, 0.0);
        let p1 = Point3::new(1.0, 0.0, 0.0);
        let p2 = Point3::new(0.0, 1.0, 0.0);

        let triangle1 = Triangle {
            normal: Vector3::new(0.0, 0.0, 1.0),
            points: vec![p0, p1, p2],
        };

        let triangle2 = Triangle {
            normal: Vector3::new(1.0, 0.0, 0.0),
            points: vec![
                Point3::new(0.0, 1.0, 1.0),
                Point3::new(0.0, 1.0, -1.0),
                Point3::new(0.0, -1.0, 0.0),
            ],
        };

        let relation = triangle1.test(&triangle2);

        let front = vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        ];

        let behind = vec![
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        ];

        assert_eq!(relation, Relation::Split { front, behind });
    }

    #[test]
    fn splits_a_triangle_where_one_point_is_in_front_and_two_are_behind() {
        let p0 = Point3::new(-1.0, 0.0, 0.0);
        let p1 = Point3::new(1.0, 0.0, 0.0);
        let p2 = Point3::new(0.0, 1.0, 0.0);

        let triangle1 = Triangle {
            normal: Vector3::new(0.0, 0.0, 1.0),
            points: vec![p0, p1, p2],
        };

        let triangle2 = Triangle {
            normal: Vector3::new(0.0, 1.0, 0.0),
            points: vec![
                Point3::new(-1.0, 0.5, -1.0),
                Point3::new(1.0, 0.5, -1.0),
                Point3::new(0.0, 0.5, 1.0),
            ],
        };

        let relation = triangle1.test(&triangle2);

        let front = vec![
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(-0.5, 0.5, 0.0),
            Point3::new(0.5, 0.5, 0.0),
        ];

        let behind = vec![
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(-0.5, 0.5, 0.0),
            Point3::new(0.5, 0.5, 0.0),
        ];

        assert_eq!(relation, Relation::Split { front, behind });
    }

    #[test]
    fn splits_a_triangle_where_two_points_are_in_front_and_one_is_behind() {
        let p0 = Point3::new(-1.0, 0.0, 0.0);
        let p1 = Point3::new(1.0, 0.0, 0.0);
        let p2 = Point3::new(0.0, 1.0, 0.0);

        let triangle1 = Triangle {
            normal: Vector3::new(0.0, 0.0, 1.0),
            points: vec![p0, p1, p2],
        };

        let triangle2 = Triangle {
            normal: Vector3::new(0.0, -1.0, 0.0),
            points: vec![
                Point3::new(-1.0, 0.5, -1.0),
                Point3::new(1.0, 0.5, -1.0),
                Point3::new(0.0, 0.5, 1.0),
            ],
        };

        let relation = triangle1.test(&triangle2);

        let front = vec![
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(-0.5, 0.5, 0.0),
            Point3::new(0.5, 0.5, 0.0),
        ];

        let behind = vec![
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(-0.5, 0.5, 0.0),
            Point3::new(0.5, 0.5, 0.0),
        ];

        assert_eq!(relation, Relation::Split { front, behind });
    }

    #[test]
    fn when_one_is_coplanar_and_two_are_behind() {
        let p0 = Point3::new(-1.0, 0.0, 0.0);
        let p1 = Point3::new(1.0, 0.0, 0.0);
        let p2 = Point3::new(0.0, 1.0, 0.0);

        let triangle1 = Triangle {
            normal: Vector3::new(0.0, 0.0, 1.0),
            points: vec![p0, p1, p2],
        };

        let triangle2 = Triangle {
            normal: Vector3::new(0.0, 1.0, 0.0),
            points: vec![
                Point3::new(-1.0, 1.0, -1.0),
                Point3::new(1.0, 1.0, -1.0),
                Point3::new(0.0, 1.0, 1.0),
            ],
        };

        let relation = triangle1.test(&triangle2);

        assert_eq!(relation, Relation::Behind);
    }

    #[test]
    fn when_one_is_coplanar_and_two_are_in_front() {
        let p0 = Point3::new(-1.0, 0.0, 0.0);
        let p1 = Point3::new(1.0, 0.0, 0.0);
        let p2 = Point3::new(0.0, 1.0, 0.0);

        let triangle1 = Triangle {
            normal: Vector3::new(0.0, 0.0, 1.0),
            points: vec![p0, p1, p2],
        };

        let triangle2 = Triangle {
            normal: Vector3::new(0.0, -1.0, 0.0),
            points: vec![
                Point3::new(-1.0, 1.0, -1.0),
                Point3::new(1.0, 1.0, -1.0),
                Point3::new(0.0, 1.0, 1.0),
            ],
        };

        let relation = triangle1.test(&triangle2);

        assert_eq!(relation, Relation::Front);
    }

    #[test]
    fn when_two_are_coplanar_and_one_is_behind() {
        let p0 = Point3::new(-1.0, 0.0, 0.0);
        let p1 = Point3::new(1.0, 0.0, 0.0);
        let p2 = Point3::new(0.0, 1.0, 0.0);

        let triangle1 = Triangle {
            normal: Vector3::new(0.0, 0.0, 1.0),
            points: vec![p0, p1, p2],
        };

        let triangle2 = Triangle {
            normal: Vector3::new(1.0, 1.0, 0.0),
            points: vec![
                Point3::new(0.0, 1.0, 1.0),
                Point3::new(0.0, 1.0, -1.0),
                Point3::new(1.0, 0.0, 0.0),
            ],
        };

        let relation = triangle1.test(&triangle2);

        assert_eq!(relation, Relation::Behind);
    }

    #[test]
    fn when_two_are_coplanar_and_one_is_in_front() {
        let p0 = Point3::new(-1.0, 0.0, 0.0);
        let p1 = Point3::new(1.0, 0.0, 0.0);
        let p2 = Point3::new(0.0, 1.0, 0.0);

        let triangle1 = Triangle {
            normal: Vector3::new(0.0, 0.0, 1.0),
            points: vec![p0, p1, p2],
        };

        let triangle2 = Triangle {
            normal: Vector3::new(-1.0, -1.0, 0.0),
            points: vec![
                Point3::new(0.0, 1.0, 1.0),
                Point3::new(0.0, 1.0, -1.0),
                Point3::new(1.0, 0.0, 0.0),
            ],
        };

        let relation = triangle1.test(&triangle2);

        assert_eq!(relation, Relation::Front);
    }

    #[test]
    fn dot_is_ordered_front_to_back() {
        assert!(Dot::Front < Dot::Coplanar);
        assert!(Dot::Front < Dot::Behind);
        assert!(Dot::Coplanar < Dot::Behind);
        assert_ne!(Dot::Front, Dot::Coplanar);
        assert_ne!(Dot::Front, Dot::Behind);
        assert_ne!(Dot::Behind, Dot::Coplanar);
    }
}
