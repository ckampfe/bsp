//# https://en.wikipedia.org/wiki/Binary_space_partitioning

use nalgebra::{Point3, Vector3};
use std::collections::VecDeque;

#[derive(Clone, Debug)]
pub struct BSPTree {
    nodes: Vec<Node>,
}

impl BSPTree {
    pub fn new(triangles: Vec<Triangle>) -> Self {
        let mut nodes: Vec<Node> = Vec::with_capacity(triangles.len());

        let mut triangles_queue = VecDeque::new();

        // initial node!
        let partition_return = partition(triangles);
        let i = nodes.len();

        if !partition_return.front.is_empty() {
            triangles_queue.push_back(NodeAction::Front {
                triangles: partition_return.front,
                node_index: i,
            });
        }

        if !partition_return.behind.is_empty() {
            triangles_queue.push_back(NodeAction::Behind {
                triangles: partition_return.behind,
                node_index: i,
            });
        }
        nodes.push(partition_return.node);

        // the recursion bit
        while let Some(node_action) = triangles_queue.pop_front() {
            match node_action {
                NodeAction::Front {
                    triangles,
                    node_index,
                } => {
                    let nodes_len = nodes.len();
                    if let Some(last_node) = nodes.get_mut(node_index) {
                        last_node.node_in_front = Some(nodes_len)
                    }

                    let partition_return = partition(triangles);
                    let i = nodes.len();

                    if !partition_return.front.is_empty() {
                        triangles_queue.push_back(NodeAction::Front {
                            triangles: partition_return.front,
                            node_index: i,
                        });
                    }

                    if !partition_return.behind.is_empty() {
                        triangles_queue.push_back(NodeAction::Behind {
                            triangles: partition_return.behind,
                            node_index: i,
                        });
                    }

                    nodes.push(partition_return.node);
                }
                NodeAction::Behind {
                    triangles,
                    node_index,
                } => {
                    let nodes_len = nodes.len();
                    if let Some(last_node) = nodes.get_mut(node_index) {
                        last_node.node_behind = Some(nodes_len)
                    }

                    let partition_return = partition(triangles);
                    let i = nodes.len();

                    if !partition_return.front.is_empty() {
                        triangles_queue.push_back(NodeAction::Front {
                            triangles: partition_return.front,
                            node_index: i,
                        });
                    }

                    if !partition_return.behind.is_empty() {
                        triangles_queue.push_back(NodeAction::Behind {
                            triangles: partition_return.behind,
                            node_index: i,
                        });
                    }
                    nodes.push(partition_return.node);
                }
            }
        }

        Self { nodes }
    }

    pub fn back_to_front_iter(&self, vantage_point: &Point3<f32>) -> BackToFrontIterator {
        let root = &self.nodes[0];
        let mut triangles = vec![];

        render(root, &self.nodes, vantage_point, &mut triangles);

        BackToFrontIterator { triangles, i: 0 }
    }
}

fn render(
    node: &Node,
    nodes: &[Node],
    vantage_point: &Point3<f32>,
    triangles_buf: &mut Vec<Triangle>,
) -> Option<Node> {
    match node.rel(vantage_point) {
        Dot::Front => {
            if let Some(behind) = node.node_behind {
                render(&nodes[behind], nodes, vantage_point, triangles_buf);
            }
            triangles_buf.extend_from_slice(&node.triangles);
            if let Some(front) = node.node_in_front {
                render(&nodes[front], nodes, vantage_point, triangles_buf);
            }
        }
        Dot::Behind => {
            if let Some(front) = node.node_in_front {
                render(&nodes[front], nodes, vantage_point, triangles_buf);
            }
            triangles_buf.extend_from_slice(&node.triangles);
            if let Some(behind) = node.node_behind {
                render(&nodes[behind], nodes, vantage_point, triangles_buf);
            }
        }
        Dot::Coplanar => {
            if let Some(front) = node.node_in_front {
                render(&nodes[front], nodes, vantage_point, triangles_buf);
            }
            if let Some(behind) = node.node_behind {
                render(&nodes[behind], nodes, vantage_point, triangles_buf);
            }
        }
    }

    None
}

struct PartitionReturn {
    node: Node,
    front: Vec<Triangle>,
    behind: Vec<Triangle>,
}

fn partition(triangles: Vec<Triangle>) -> PartitionReturn {
    let initial_triangle_idx = triangles.len() / 2;
    let mut triangles = triangles;
    let p = triangles.remove(initial_triangle_idx);

    let mut n = Node::new();

    let mut front_return = vec![];
    let mut behind_return = vec![];

    for tri in triangles.into_iter() {
        let relation = tri.test(&p);

        match relation {
            Relation::Front => {
                front_return.push(tri);
            }
            Relation::Behind => {
                behind_return.push(tri);
            }
            Relation::Coplanar => {
                n.add_triangle_at(tri);
            }
            Relation::Split { front, behind } => {
                front_return.extend_from_slice(&front);
                behind_return.extend_from_slice(&behind);
            }
        }
    }

    n.add_triangle_at(p);

    PartitionReturn {
        node: n,
        front: front_return,
        behind: behind_return,
    }
}

enum NodeAction {
    Front {
        triangles: Vec<Triangle>,
        node_index: usize,
    },
    Behind {
        triangles: Vec<Triangle>,
        node_index: usize,
    },
}

pub struct BackToFrontIterator {
    triangles: Vec<Triangle>,
    i: usize,
}

impl Iterator for BackToFrontIterator {
    type Item = Triangle;

    fn next(&mut self) -> Option<Self::Item> {
        if let Some(t) = self.triangles.get(self.i) {
            let r = Some(t.to_owned());
            self.i += 1;
            r
        } else {
            None
        }
    }
}

#[derive(Clone, Debug)]
struct Node {
    triangles: Vec<Triangle>,
    node_in_front: Option<usize>,
    node_behind: Option<usize>,
}

impl Node {
    fn new() -> Self {
        Self {
            triangles: vec![],
            node_in_front: None,
            node_behind: None,
        }
    }

    fn add_triangle_at(&mut self, triangle: Triangle) {
        self.triangles.push(triangle);
    }

    fn rel(&self, vantage_point: &Point3<f32>) -> Dot {
        self.triangles[0].normal.dot(&vantage_point.coords).rel()
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Triangle {
    normal: Vector3<f32>,
    points: Vec<Point3<f32>>,
}

impl Triangle {
    pub fn new(points: Vec<Point3<f32>>, normal: Vector3<f32>) -> Triangle {
        Self { normal, points }
    }

    pub fn normal(&self) -> Vector3<f32> {
        self.normal
    }

    pub fn points(&self) -> &[Point3<f32>] {
        self.points.as_slice()
    }

    fn test(&self, other: &Triangle) -> Relation {
        self.split(other)
    }

    // TODO figure out how to make sure points are:
    // 1. ordered
    // 2. unique
    fn split(&self, plane: &Triangle) -> Relation {
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
                // results in 3 total triangles, one in front, and two behind

                let p0p1i = ray_to_plane_intersection(&(p0, p1), &plane.normal(), &plane.points()[0]);
                let p0p2i = ray_to_plane_intersection(&(p0, p2), &plane.normal(), &plane.points()[0]);

                let front_triangle = Triangle::new(vec![p0, p0p1i, p0p2i], self.normal());

                let behind_triangle_1 = Triangle::new(vec![p1, p2, p0p1i], self.normal());
                let behind_triangle_2 = Triangle::new(vec![p0p1i, p0p2i, p2], self.normal());

                Relation::Split {
                    front: vec![front_triangle], behind: vec![behind_triangle_1, behind_triangle_2]
                }
            }
            [(Dot::Front, p0), (Dot::Front, p1), (Dot::Behind, p2)] => {
                // case where plane intersects no points, with two in front and one behind
                // results in 3 total triangles, two in front and one behind

                let p0p2i = ray_to_plane_intersection(&(p0, p2), &plane.normal(), &plane.points()[0]);
                let p1p2i = ray_to_plane_intersection(&(p1, p2), &plane.normal(), &plane.points()[0]);

                let front_triangle_1 = Triangle::new(vec![p0, p1, p0p2i], self.normal());
                let front_triangle_2 = Triangle::new(vec![p0, p1p2i, p0p2i], self.normal());

                let behind_triangle = Triangle::new(vec![p2, p0p2i, p1p2i], self.normal());

                Relation::Split {
                    front: vec![front_triangle_1, front_triangle_2], behind: vec![behind_triangle]
                }
            }
            [(Dot::Front, p0), (Dot::Coplanar, p1), (Dot::Behind, p2)] => {
                // case where plane intersects one point, with one in front and one behind
                // results in two total triangles, one in front and one behind

                let p0p2i= ray_to_plane_intersection(&(p0, p2), &plane.normal(), &plane.points()[0]);

                let front_triangle = Triangle::new(vec![p0, p1, p0p2i], self.normal());

                let behind_triangle = Triangle::new(vec![p1, p2, p0p2i], self.normal());

                Relation::Split {
                    front: vec![front_triangle], behind: vec![behind_triangle]
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
pub enum Relation {
    Front,
    Behind,
    Coplanar,
    Split {
        front: Vec<Triangle>,
        behind: Vec<Triangle>,
    },
}

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
    use rand::seq::SliceRandom;
    use rand::thread_rng;

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

        let front_points = vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        ];

        let front = vec![Triangle::new(front_points, Vector3::new(0.0, 0.0, 1.0))];

        let behind_points = vec![
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        ];

        let behind = vec![Triangle::new(behind_points, Vector3::new(0.0, 0.0, 1.0))];

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

        let front_points = vec![
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(-0.5, 0.5, 0.0),
            Point3::new(0.5, 0.5, 0.0),
        ];

        let front = vec![Triangle::new(front_points, Vector3::new(0.0, 0.0, 1.0))];

        let behind_points = vec![
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(-0.5, 0.5, 0.0),
            Point3::new(0.5, 0.5, 0.0),
        ];

        let behind = vec![
            Triangle::new(
                vec![behind_points[0], behind_points[1], behind_points[2]],
                Vector3::new(0.0, 0.0, 1.0),
            ),
            Triangle::new(
                vec![behind_points[2], behind_points[3], behind_points[1]],
                Vector3::new(0.0, 0.0, 1.0),
            ),
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

        let behind_points = vec![
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(-0.5, 0.5, 0.0),
            Point3::new(0.5, 0.5, 0.0),
        ];

        let behind = vec![Triangle::new(behind_points, Vector3::new(0.0, 0.0, 1.0))];

        let front_points = vec![
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(-0.5, 0.5, 0.0),
            Point3::new(0.5, 0.5, 0.0),
        ];

        let front = vec![
            Triangle::new(
                vec![front_points[0], front_points[1], front_points[2]],
                Vector3::new(0.0, 0.0, 1.0),
            ),
            Triangle::new(
                vec![front_points[0], front_points[3], front_points[2]],
                Vector3::new(0.0, 0.0, 1.0),
            ),
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

    #[test]
    fn triangles_are_ordered_back_to_front() {
        let t1 = Triangle::new(
            vec![
                Point3::new(-1.0, 1.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 1.0),
            ],
            Vector3::new(0.0, -1.0, 0.0),
        );
        let t2 = Triangle::new(
            vec![
                Point3::new(-1.0, 2.0, 0.0),
                Point3::new(1.0, 2.0, 0.0),
                Point3::new(0.0, 2.0, 1.0),
            ],
            Vector3::new(0.0, -1.0, 0.0),
        );
        let t3 = Triangle::new(
            vec![
                Point3::new(-1.0, 3.0, 0.0),
                Point3::new(1.0, 3.0, 0.0),
                Point3::new(0.0, 3.0, 1.0),
            ],
            Vector3::new(0.0, -1.0, 0.0),
        );
        let t4 = Triangle::new(
            vec![
                Point3::new(-1.0, 4.0, 0.0),
                Point3::new(1.0, 4.0, 0.0),
                Point3::new(0.0, 4.0, 1.0),
            ],
            Vector3::new(0.0, -1.0, 0.0),
        );
        let t5 = Triangle::new(
            vec![
                Point3::new(-1.0, 5.0, 0.0),
                Point3::new(1.0, 5.0, 0.0),
                Point3::new(0.0, 5.0, 1.0),
            ],
            Vector3::new(0.0, -1.0, 0.0),
        );

        let mut triangles = vec![t1.clone(), t2.clone(), t3.clone(), t4.clone(), t5.clone()];
        let mut rng = thread_rng();
        triangles.shuffle(&mut rng);

        let tree = BSPTree::new(triangles);

        let btf = tree
            .back_to_front_iter(&Point3::new(0.0, -5.0, 0.0))
            .collect::<Vec<Triangle>>();

        assert_eq!(
            btf,
            vec![t5.clone(), t4.clone(), t3.clone(), t2.clone(), t1.clone()]
        );

        let btf2 = tree
            .back_to_front_iter(&Point3::new(0.0, 10.0, 0.0))
            .collect::<Vec<Triangle>>();

        assert_eq!(btf2, vec![t1, t2, t3, t4, t5]);
    }
}
