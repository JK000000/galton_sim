use std::cell::RefCell;
use std::cmp::{max, Ordering, Reverse};
use std::rc::Rc;
use glam::DVec2;
use priority_queue::PriorityQueue;
use rustc_hash::{FxHashMap, FxHashSet};
use crate::core::shapes::Shape;
use crate::core::tree::BoundingBoxTreeChildren::{NodeChildren, ShapeChild};

#[derive(Clone)]
pub struct BoundingBox {
    pub(crate) point_low: DVec2,
    pub(crate) point_high: DVec2,
}

impl BoundingBox {
    #[inline(always)]
    pub fn does_collide(&self, other: &BoundingBox) -> bool {
        if self.point_high.x < other.point_low.x {
            return false;
        }
        if self.point_high.y < other.point_low.y {
            return false;
        }
        if self.point_low.x > other.point_high.x {
            return false;
        }
        if self.point_low.y > other.point_high.y {
            return false;
        }
        true
    }

    #[inline(always)]
    pub fn join(a: &BoundingBox, b: &BoundingBox) -> BoundingBox {
        BoundingBox {
            point_low: DVec2 {
                x: f64::min(a.point_low.x, b.point_low.x),
                y: f64::min(a.point_low.y, b.point_low.y),
            },
            point_high: DVec2 {
                x: f64::max(a.point_high.x, b.point_high.x),
                y: f64::max(a.point_high.y, b.point_high.y),
            },
        }
    }

    #[inline(always)]
    pub fn join_in_place(&mut self, other: &BoundingBox) {
        self.point_low.x = f64::min(self.point_low.x, other.point_low.x);
        self.point_low.y = f64::min(self.point_low.y, other.point_low.y);
        self.point_high.x = f64::min(self.point_high.x, other.point_high.x);
        self.point_high.y = f64::min(self.point_high.y, other.point_high.y);
    }

    #[inline(always)]
    pub fn intersect(a: &BoundingBox, b: &BoundingBox) -> Option<BoundingBox> {
        let res = BoundingBox {
            point_low: DVec2 {
                x: f64::max(a.point_low.x, b.point_low.x),
                y: f64::max(a.point_low.y, b.point_low.y),
            },
            point_high: DVec2 {
                x: f64::min(a.point_high.x, b.point_high.x),
                y: f64::min(a.point_high.y, b.point_high.y),
            },
        };

        if res.point_low.x > res.point_high.x || res.point_low.y > res.point_high.y {
            return None;
        }

        Some(res)
    }

    #[inline(always)]
    pub fn does_intersect(a: &BoundingBox, b: &BoundingBox) -> bool {
        !(f64::max(a.point_low.x, b.point_low.x) > f64::min(a.point_high.x, b.point_high.x) || f64::max(a.point_low.y, b.point_low.y) > f64::min(a.point_high.y, b.point_high.y))
    }

    #[inline(always)]
    pub fn extended(&self, ratio: f64) -> BoundingBox {
        let len_x = self.point_high.x - self.point_low.x;
        let len_y = self.point_high.y - self.point_low.y;
        BoundingBox {
            point_high: DVec2::new(self.point_high.x + len_x * ratio, self.point_high.y + len_y * ratio),
            point_low: DVec2::new(self.point_low.x - len_x * ratio, self.point_low.y - len_y * ratio),
        }
    }

    #[inline(always)]
    pub fn surface_area(&self) -> f64 {
        (self.point_high.x - self.point_low.x) * (self.point_high.y - self.point_low.y)
    }

    #[inline(always)]
    pub fn center(&self) -> DVec2 {
        (self.point_low + self.point_high) / 2.0
    }

    #[inline(always)]
    pub fn is_inside(&self, other: &BoundingBox) -> bool {
        if self.point_low.x < other.point_low.x || self.point_low.y < other.point_low.y {
            return false;
        }

        if self.point_high.x > other.point_high.x || self.point_high.y > other.point_high.y {
            return false;
        }

        return true;
    }
}

// Only some shapes are allowed to change and we need to keep track of
// whether node's children contain movable shapes in order to allow for efficient update
struct BoundingBoxTreeNode {
    bounding_box: BoundingBox,
    children: BoundingBoxTreeChildren,
    contains_movable: bool,
    quality_total_negative_score: f64,
}

enum BoundingBoxTreeChildren {
    ShapeChild(LeafShapeData),
    NodeChildren([Box<BoundingBoxTreeNode>; 2]),
}

#[derive(Clone)]
struct LeafShapeData {
    shape: Rc<RefCell<Shape>>,
    object_id: usize,
    is_movable: bool,
}

#[derive(Clone)]
pub struct TreeSearchResult {
    pub shape: Rc<RefCell<Shape>>,
    pub object_id: usize,
}

// Struct to hold the tree
pub struct BoundingBoxTree {
    root: Option<Box<BoundingBoxTreeNode>>,
    tree_built: bool,
    quality_last_build: f64,
    leaves: FxHashMap<usize, LeafShapeData>,
    recently_deleted_ids: FxHashSet<usize>,
    updates_without_rebuild: usize
}

// BEGIN Helper functions for operation of the tree

// Returns true if b has greater most significant bit than a
fn less_msb(a: u64, b: u64) -> bool {
    a < b && (a < (a ^ b))
}

fn to_z_order_coords(x: f64) -> u64 {
    let mut res = x;
    res += 1e6;
    res *= 1e3;
    if res < 0.0 {
        0
    } else {
        res.round() as u64
    }
}

// Function intended for use when sorting array of tree nodes with respect to z-order
fn z_order_box_cmp(a: &BoundingBox, b: &BoundingBox) -> Ordering {
    let mut ca: [u64; 2] = [0; 2];
    let mut cb: [u64; 2] = [0; 2];

    ca[0] = to_z_order_coords((a.point_low.x + a.point_high.x) / 2.0);
    ca[1] = to_z_order_coords((a.point_low.y + a.point_high.y) / 2.0);

    cb[0] = to_z_order_coords((b.point_low.x + b.point_high.x) / 2.0);
    cb[1] = to_z_order_coords((b.point_low.y + b.point_high.y) / 2.0);

    if ca == cb {
        return Ordering::Equal;
    }

    let mut most_significant_dim = 1;


    if less_msb(ca[1] ^ cb[1], ca[0] ^ cb[0]) {
        most_significant_dim = 0;
    }


    if ca[most_significant_dim] < cb[most_significant_dim] {
        Ordering::Less
    } else {
        Ordering::Greater
    }
}

fn get_score_for_quality(a: &BoundingBox, b: &BoundingBox) -> f64 {
    let res_area = BoundingBox::join(a, b).surface_area();

    let mut children_area = a.surface_area() + b.surface_area();

    if let Some(intersection) = BoundingBox::intersect(a, b) {
        children_area -= 1.25 * intersection.surface_area();
    }

    res_area - children_area
}

fn max_acceptable_quality(curr_quality: f64) -> f64 {
    curr_quality * 1.05 + 0.15
}

// END   Helper functions for operation of the tree


/*
 * Tree generation implemented here is based on the paper:
 * Yan Gu, Yong He, Kayvon Fatahalian, and Guy Blelloch. 2013.
 * Efficient BVH construction via approximate agglomerative clustering.
 * In Proceedings of the 5th High-Performance Graphics Conference (HPG '13).
 */

// BEGIN Parameters for the AAC algorithm
const DELTA: usize = 10; // Traversal stopping threshold

// Cluster count reduction function
fn f(x: usize) -> usize {
    (x as f64).powf(0.5).ceil() as usize
}

// Function used to evaluate how bad is it to join clusters with given bounding boxes
fn get_score(a: &BoundingBox, b: &BoundingBox) -> u64 {
    let res_area = BoundingBox::join(a, b).surface_area();

    let mut children_area = a.surface_area() + b.surface_area();

    if let Some(intersection) = BoundingBox::intersect(a, b) {
        children_area -= intersection.surface_area();
    }

    let ratio = children_area / res_area;
    let mult = 1.0 / (1.0 + ratio);

    (res_area * mult * 1000.0).round() as u64
}
// END   Parameters for the AAC algorithm


// BEGIN Functions of the AAC algorithm

struct ClustersHolder {
    clusters: FxHashMap<usize, Box<BoundingBoxTreeNode>>,
    next_id: usize,
}

impl ClustersHolder {
    fn z_order_split(self) -> (ClustersHolder, ClustersHolder) {
        let mut idxs: Vec<usize> = self.clusters.keys().map(|x| *x).collect();

        idxs.sort_unstable_by(|a, b| z_order_box_cmp(&self.clusters.get(a).unwrap().bounding_box, &self.clusters.get(b).unwrap().bounding_box));

        let mut first_half = FxHashSet::default();

        for i in 0..(idxs.len() / 2) {
            first_half.insert(idxs[i]);
        }

        let mut left = FxHashMap::default();
        let mut right = FxHashMap::default();

        for (id, node) in self.clusters.into_iter() {
            if first_half.contains(&id) {
                left.insert(id, node);
            } else {
                right.insert(id, node);
            }
        }

        let left_max_num_ids = left.len() * 2;

        (
            ClustersHolder {
                clusters: left,
                next_id: self.next_id,
            },
            ClustersHolder {
                clusters: right,
                next_id: self.next_id + left_max_num_ids,
            }
        )
    }

    fn join(&mut self, other: ClustersHolder) {
        self.next_id = max(self.next_id, other.next_id);

        for (i, c) in other.clusters.into_iter() {
            self.clusters.insert(i, c);
        }
    }
}

// Combines given clusters in an optimal fashion until 'n' clusters remain.
fn combine_clusters(holder: &mut ClustersHolder, n: usize) {
    assert!(n >= 1);


    let mut scores_queue: PriorityQueue<(usize, usize), Reverse<u64>> = PriorityQueue::new();

    for (i, ci) in holder.clusters.iter() {
        for (j, cj) in holder.clusters.iter() {
            if *i == *j {
                continue;
            }

            scores_queue.push((*i, *j), Reverse(get_score(&ci.bounding_box, &cj.bounding_box)));
        }
    }


    while holder.clusters.len() > n {
        let Some((node_pair, _score)) = scores_queue.pop() else { panic!() };
        let (node_a_id, node_b_id) = node_pair;

        if !holder.clusters.contains_key(&node_a_id) || !holder.clusters.contains_key(&node_b_id) {
            continue;
        }

        let node_a: Box<BoundingBoxTreeNode> = holder.clusters.remove(&node_a_id).unwrap();
        let node_b: Box<BoundingBoxTreeNode> = holder.clusters.remove(&node_b_id).unwrap();

        let quality_score = get_score_for_quality(&node_a.bounding_box, &node_b.bounding_box);

        let contains_movable = node_a.contains_movable || node_b.contains_movable;
        let quality_total_negative_score = quality_score + node_a.quality_total_negative_score + node_b.quality_total_negative_score;

        let new_node = Box::new(BoundingBoxTreeNode {
            bounding_box: BoundingBox::join(&node_a.bounding_box, &node_b.bounding_box),
            children: BoundingBoxTreeChildren::NodeChildren([node_a, node_b]),
            contains_movable,
            quality_total_negative_score,
        });


        let next_id = holder.next_id;
        holder.next_id += 1;


        for (i, cl) in holder.clusters.iter() {
            let i = *i;
            scores_queue.push((i, next_id), Reverse(get_score(&new_node.bounding_box, &cl.bounding_box)));
        }

        holder.clusters.insert(next_id, new_node);
    }
}

fn build_tree(mut clusters: ClustersHolder) -> ClustersHolder {
    if clusters.clusters.len() <= DELTA {
        combine_clusters(&mut clusters, f(DELTA));
        return clusters;
    }

    let (l, r) = clusters.z_order_split();

    let mut l = build_tree(l);
    let r = build_tree(r);

    l.join(r);

    let n = l.clusters.len();
    combine_clusters(&mut l, f(n));
    l
}

// END   Functions of the AAC algorithm

impl BoundingBoxTreeNode {
    fn add_leaf(&mut self, leaf: &LeafShapeData, shape_center: &DVec2, shape_box: &BoundingBox) {
        if let NodeChildren([a, b]) = &mut self.children {
            if shape_center.distance(a.bounding_box.center()) < shape_center.distance(b.bounding_box.center()) {
                a.add_leaf(leaf, shape_center, shape_box);
            } else {
                b.add_leaf(leaf, shape_center, shape_box);
            }

            self.contains_movable = a.contains_movable || b.contains_movable;
            self.quality_total_negative_score = get_score_for_quality(&a.bounding_box, &b.bounding_box);
            self.bounding_box = BoundingBox::join(&a.bounding_box, &b.bounding_box);
        } else {
            let new_node_child0 = Box::new(BoundingBoxTreeNode {
                bounding_box: shape_box.clone(),
                children: BoundingBoxTreeChildren::ShapeChild(leaf.clone()),
                contains_movable: leaf.is_movable,
                quality_total_negative_score: 0.0,
            });

            let ShapeChild(m_shape) = &self.children else { panic!() };

            let new_node_child1 = Box::new(BoundingBoxTreeNode {
                bounding_box: self.bounding_box.clone(),
                children: BoundingBoxTreeChildren::ShapeChild(m_shape.clone()),
                contains_movable: self.contains_movable,
                quality_total_negative_score: 0.0,
            });

            self.contains_movable |= leaf.is_movable;
            self.bounding_box.join_in_place(shape_box);
            self.quality_total_negative_score = get_score_for_quality(&new_node_child0.bounding_box, &new_node_child1.bounding_box);
            self.children = NodeChildren([new_node_child0, new_node_child1]);
        }
    }
}

impl BoundingBoxTree {

    const MAX_UPDATES_WITHOUT_REBUILD: usize = 1000;
    pub fn new() -> BoundingBoxTree {
        BoundingBoxTree {
            root: None,
            tree_built: false,
            quality_last_build: 0.0,
            leaves: FxHashMap::default(),
            recently_deleted_ids: FxHashSet::default(),
            updates_without_rebuild: 0
        }
    }

    pub fn add_shape(&mut self, body_id: usize, is_movable: bool, shape: &Rc<RefCell<Shape>>) {
        let leaf = LeafShapeData {
            shape: Rc::clone(shape),
            object_id: body_id,
            is_movable,
        };

        if self.tree_built {
            let shape_box = shape.borrow().get_bounding_box();
            let shape_center = shape_box.center();

            self.root.as_mut().unwrap().add_leaf(&leaf, &shape_center, &shape_box);
        }

        self.leaves.insert(leaf.object_id, leaf);
    }

    fn build(&mut self) {
        assert!(!self.tree_built);

        self.recently_deleted_ids.clear();

        let initial_clusters: FxHashMap<usize, Box<BoundingBoxTreeNode>> = self.leaves.iter().map(|(_, x)| {
            let is_movable = x.is_movable;
            let bounding_box = x.shape.borrow().get_bounding_box();

            Box::new(BoundingBoxTreeNode {
                bounding_box,
                contains_movable: is_movable,
                quality_total_negative_score: 0.0,
                children: ShapeChild(x.clone()),
            })
        }).enumerate().collect();
        let n = initial_clusters.len();

        let initial_clusters = ClustersHolder {
            clusters: initial_clusters,
            next_id: n,
        };

        let mut res_clusters = build_tree(initial_clusters);
        combine_clusters(&mut res_clusters, 1);

        let (_, root) = res_clusters.clusters.into_iter().last().unwrap();
        self.root = Some(root);
        self.tree_built = true;
    }

    fn update_boxes_and_quality_dfs(node: &mut BoundingBoxTreeNode) {
        if !node.contains_movable {
            return;
        }

        match &mut node.children {
            BoundingBoxTreeChildren::ShapeChild(data) => {
                node.bounding_box = data.shape.borrow().get_bounding_box();
            }
            BoundingBoxTreeChildren::NodeChildren([a, b]) => {
                BoundingBoxTree::update_boxes_and_quality_dfs(a.as_mut());
                BoundingBoxTree::update_boxes_and_quality_dfs(b.as_mut());

                node.bounding_box = BoundingBox::join(&a.bounding_box, &b.bounding_box);
                node.quality_total_negative_score = get_score_for_quality(&a.bounding_box, &b.bounding_box) + a.quality_total_negative_score + b.quality_total_negative_score;
            }
        }
    }

    fn update_boxes_and_quality(&mut self) {
        BoundingBoxTree::update_boxes_and_quality_dfs(self.root.as_mut().unwrap().as_mut());
    }

    fn quality(&self) -> f64 {
        assert!(self.tree_built);

        let root_wasted_space = self.root.as_ref().unwrap().quality_total_negative_score;

        let root_space = self.root.as_ref().unwrap().bounding_box.surface_area();

        let waste_factor = root_wasted_space / root_space;

        waste_factor
    }

    pub fn update(&mut self) {
        if self.tree_built {
            if self.updates_without_rebuild < BoundingBoxTree::MAX_UPDATES_WITHOUT_REBUILD {
                self.update_boxes_and_quality();
                let curr_quality = self.quality();

                if curr_quality <= max_acceptable_quality(self.quality_last_build) {
                    self.updates_without_rebuild += 1;
                    return;
                }
            }

            self.tree_built = false;
            self.root = None;
        }

        self.build();
        self.quality_last_build = self.quality();
        self.updates_without_rebuild = 0;
    }

    pub fn delete(&mut self, id: usize) {
        self.recently_deleted_ids.insert(id);
        self.leaves.remove(&id);
    }

    fn shapes_in_box_dfs(&self, search_box: &BoundingBox, node: &BoundingBoxTreeNode, on_found: &mut impl FnMut(TreeSearchResult)) {
        match &node.children {
            BoundingBoxTreeChildren::ShapeChild(data) => {
                if !self.recently_deleted_ids.contains(&data.object_id) {
                    on_found(TreeSearchResult {
                        object_id: data.object_id,
                        shape: data.shape.clone(),
                    });
                }
            }
            BoundingBoxTreeChildren::NodeChildren([a, b]) => {
                if BoundingBox::does_intersect(&a.bounding_box, search_box) {
                    self.shapes_in_box_dfs(search_box, a.as_ref(), on_found);
                }
                if BoundingBox::does_intersect(&b.bounding_box, search_box) {
                    self.shapes_in_box_dfs(search_box, b.as_ref(), on_found);
                }
            }
        }
    }

    pub fn shapes_in_box(&self, search_box: &BoundingBox, on_found: &mut impl FnMut(TreeSearchResult)) {
        assert!(self.tree_built);

        self.shapes_in_box_dfs(search_box, self.root.as_ref().unwrap().as_ref(), on_found);

    }
}

#[cfg(test)]
mod tests {
    use std::fmt::{Display, Formatter};
    use std::ops::DerefMut;
    use crate::core::shapes::Shape::DebugBox;
    use super::*;

    #[test]
    fn bounding_box() {
        let a = BoundingBox {
            point_low: DVec2 {x: 1.0, y: 2.0},
            point_high: DVec2 {x: 2.0, y: 4.0}
        };

        let b = BoundingBox {
            point_low: DVec2 {x: -1.0, y: 3.0},
            point_high: DVec2 {x: 2.0, y: 3.5}
        };

        let c = BoundingBox::join(&a, &b);

        assert_eq!(c.point_low.x, -1.0);
        assert_eq!(c.point_low.y, 2.0);
        assert_eq!(c.point_high.x, 2.0);
        assert_eq!(c.point_high.y, 4.0);

        assert!(a.does_collide(&b));
    }

    #[test]
    fn z_order() {
        let mut boxes = Vec::new();

        for i in 0..4 {
            for j in 0..4 {
                let point = DVec2 {x : 0.001 * i as f64 - 1e6, y: 0.001 * j as f64 - 1e6};
                boxes.push(((i,j),BoundingBox {
                    point_low: point,
                    point_high: point
                }));
            }
        }

        boxes.sort_by(|(_, a), (_, b)| z_order_box_cmp(a, b));

        let correct_order = [(0,0), (1,0), (0,1), (1,1), (2,0), (3,0), (2,1), (3,1), (0,2), (1,2), (0,3), (1,3), (2,2), (3,2), (2,3), (3,3)];

        for i in 0..16 {
            let (box_pos, _) = boxes[i];
            assert_eq!(correct_order[i], box_pos);
        }
    }

    impl Display for BoundingBoxTreeNode {
        fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
            write!(f, "[box=<({},{}),({},{})>", self.bounding_box.point_low.x, self.bounding_box.point_low.y, self.bounding_box.point_high.x, self.bounding_box.point_high.y)?;

            if let BoundingBoxTreeChildren::NodeChildren([a, b]) = &self.children {
                write!(f, ",children=<{},{}>", a.as_ref(), b.as_ref())?;
            }

            write!(f, "]")
        }
    }

    fn get_mock_boxes() -> [BoundingBox; 6] {
        [
            BoundingBox {
                point_low: DVec2 {x: 1.0, y: 0.0},
                point_high: DVec2 {x: 2.0, y: 3.0}
            },
            BoundingBox {
                point_low: DVec2 {x: 0.0, y: 2.0},
                point_high: DVec2 {x: 3.0, y: 3.0}
            },
            BoundingBox {
                point_low: DVec2 {x: 0.0, y: 5.0},
                point_high: DVec2 {x: 2.0, y: 6.0}
            },
            BoundingBox {
                point_low: DVec2 {x: 1.0, y: 4.0},
                point_high: DVec2 {x: 2.0, y: 6.0}
            },
            BoundingBox {
                point_low: DVec2 {x: 4.0, y: 4.0},
                point_high: DVec2 {x: 5.0, y: 6.0}
            },
            BoundingBox {
                point_low: DVec2 {x: 4.0, y: 2.0},
                point_high: DVec2 {x: 5.0, y: 3.0}
            }
        ]
    }

    #[test]
    fn aac_combine_clusters() {
        let mut clusters = Vec::new();

        let boxes = get_mock_boxes();

        for b in boxes {
            clusters.push(Box::new(BoundingBoxTreeNode {
                bounding_box: b.clone(),
                children: BoundingBoxTreeChildren::ShapeChild(LeafShapeData {
                    shape: Rc::new(RefCell::new(Shape::DebugBox(b.clone()))),
                    object_id: 0,
                    is_movable: false
                }),
                contains_movable: false,
                quality_total_negative_score: 0.0
            }));
        }

        let n = clusters.len();

        let mut clusters = ClustersHolder {
            clusters: clusters.into_iter().enumerate().collect(),
            next_id: n
        };

        combine_clusters(&mut clusters, 2);

        let clusters: Vec<Box<BoundingBoxTreeNode>> = clusters.clusters.into_iter().map(|(_,x)| x).collect();

        assert_eq!("[box=<(0,0),(3,6)>,children=<[box=<(0,4),(2,6)>,children=<[box=<(0,5),(2,6)>],[box=<(1,4),(2,6)>]>],[box=<(0,0),(3,3)>,children=<[box=<(1,0),(2,3)>],[box=<(0,2),(3,3)>]>]>]", format!("{}", *clusters[0]));
        assert_eq!("[box=<(4,2),(5,6)>,children=<[box=<(4,2),(5,3)>],[box=<(4,4),(5,6)>]>]", format!("{}", *clusters[1]));
    }

    #[test]
    fn build_tree_small() {
        let mut tree = BoundingBoxTree::new();

        let boxes = get_mock_boxes();

        let mut i : i32 = 0;

        for b in boxes {
            let cell =  RefCell::new(Shape::DebugBox(b));
            let cell_ref : Rc<RefCell<Shape>> = Rc::new(cell);
            tree.add_shape(i as usize, false, &cell_ref);
            i += 1;
        }

        tree.build();

        assert!(tree.quality() <= 1.0);

        assert_eq!("[box=<(0,0),(5,6)>,children=<[box=<(4,2),(5,6)>,children=<[box=<(4,2),(5,3)>],[box=<(4,4),(5,6)>]>],[box=<(0,0),(3,6)>,children=<[box=<(0,4),(2,6)>,children=<[box=<(1,4),(2,6)>],[box=<(0,5),(2,6)>]>],[box=<(0,0),(3,3)>,children=<[box=<(0,2),(3,3)>],[box=<(1,0),(2,3)>]>]>]>]",
                   format!("{}",&tree.root.unwrap()));

    }

    #[test]
    fn build_tree_large() {
        let mut tree = BoundingBoxTree::new();

        let mut boxes = Vec::new();

        for i in 0..100 {
            for j in 0..100 {
                boxes.push(BoundingBox {
                    point_low: DVec2{x: i as f64, y: j as f64},
                    point_high: DVec2{x: (i+1) as f64, y: (j+1) as f64}
                })
            }
        }

        let mut i: i32 = 0;

        for b in boxes {
            let cell =  RefCell::new(Shape::DebugBox(b));
            let cell_ref : Rc<RefCell<Shape>> = Rc::new(cell);
            tree.add_shape(i as usize, false, &cell_ref);
            i += 1;
        }

        tree.build();
        println!("{}", tree.quality());
        assert!(tree.quality() <= 2.0);
    }

    #[test]
    fn build_tree_update_boxes() {
        let mut tree = BoundingBoxTree::new();

        let boxes = get_mock_boxes();

        let mut i : i32 = -1;

        let mut shapes = Vec::new();

        for b in boxes {
            let cell_ref =  Rc::new(RefCell::new(Shape::DebugBox(b)));
            let cell_ref_shape : Rc<RefCell<Shape>> = Rc::clone(&cell_ref);
            tree.add_shape(i as usize, true, &cell_ref_shape);
            shapes.push(cell_ref);
            i -= 1;
        }

        tree.build();

        for i in shapes.iter() {
            if let Shape::DebugBox(b) = i.borrow_mut().deref_mut() {
                b.point_low.x += 1.0;
                b.point_high.x += 1.0;
            }
        }

        tree.update_boxes_and_quality();

        assert_eq!("[box=<(1,0),(6,6)>,children=<[box=<(5,2),(6,6)>,children=<[box=<(5,4),(6,6)>],[box=<(5,2),(6,3)>]>],[box=<(1,0),(4,6)>,children=<[box=<(1,4),(3,6)>,children=<[box=<(1,5),(3,6)>],[box=<(2,4),(3,6)>]>],[box=<(1,0),(4,3)>,children=<[box=<(1,2),(4,3)>],[box=<(2,0),(3,3)>]>]>]>]",
                   format!("{}",&tree.root.unwrap()));
    }

    #[test]
    fn update_tree() {
        let mut tree = BoundingBoxTree::new();

        let boxes = [
            BoundingBox {
                point_low: DVec2 {x: -4.0, y: -3.0},
                point_high: DVec2 {x: -3.0, y: -2.0}
            },
            BoundingBox {
                point_low: DVec2 {x: -3.0, y: -4.0},
                point_high: DVec2 {x: -2.0, y: -3.0}
            },
            BoundingBox {
                point_low: DVec2 {x: 3.0, y: 2.0},
                point_high: DVec2 {x: 4.0, y: 3.0}
            },
            BoundingBox {
                point_low: DVec2 {x: 2.0, y: 3.0},
                point_high: DVec2 {x: 3.0, y: 4.0}
            }
        ];

        let mut i : i32 = -1;

        let mut shapes = Vec::new();

        for b in boxes {
            let cell_ref =  Rc::new(RefCell::new(Shape::DebugBox(b)));
            let cell_ref_shape : Rc<RefCell<Shape>> = Rc::clone(&cell_ref);
            tree.add_shape(i as usize, true, &cell_ref_shape);
            shapes.push(cell_ref);
            i -= 1;
        }

        tree.update();

        assert_eq!("[box=<(-4,-4),(4,4)>,children=<[box=<(2,2),(4,4)>,children=<[box=<(3,2),(4,3)>],[box=<(2,3),(3,4)>]>],[box=<(-4,-4),(-2,-2)>,children=<[box=<(-4,-3),(-3,-2)>],[box=<(-3,-4),(-2,-3)>]>]>]", format!("{}", tree.root.as_ref().unwrap()));

        let shift_up = DVec2{x: 0.0, y: 1.0};
        let shift_down = DVec2{x: 0.0, y: -1.0};
        let shift_left = DVec2 {x: -1.0, y: 0.0};
        let shift_right = DVec2 {x: 1.0, y: 0.0};

        for _ in 0..5 {
            if let Shape::DebugBox(b) = shapes[0].borrow_mut().deref_mut() {
                b.point_low += shift_up;
                b.point_high += shift_up;
            }

            if let Shape::DebugBox(b) = shapes[1].borrow_mut().deref_mut() {
                b.point_low += shift_right;
                b.point_high += shift_right;
            }

            if let Shape::DebugBox(b) = shapes[2].borrow_mut().deref_mut() {
                b.point_low += shift_down;
                b.point_high += shift_down;
            }

            if let Shape::DebugBox(b) = shapes[3].borrow_mut().deref_mut() {
                b.point_low += shift_left;
                b.point_high += shift_left;
            }

            tree.update();

        }

        assert_eq!("[box=<(-4,-4),(4,4)>,children=<[box=<(2,-4),(4,-2)>,children=<[box=<(3,-3),(4,-2)>],[box=<(2,-4),(3,-3)>]>],[box=<(-4,2),(-2,4)>,children=<[box=<(-3,3),(-2,4)>],[box=<(-4,2),(-3,3)>]>]>]", format!("{}", tree.root.as_ref().unwrap()));
    }

    #[test]
    fn tree_shapes_in_box() {
        let mut tree = BoundingBoxTree::new();

        let mut boxes = Vec::new();

        for i in 0..100 {
            for j in 0..100 {
                boxes.push(BoundingBox {
                    point_low: DVec2{x: i as f64, y: j as f64},
                    point_high: DVec2{x: (i+1) as f64, y: (j+1) as f64}
                })
            }
        }

        let mut i: i32 = 0;

        for b in boxes {
            let cell =  RefCell::new(DebugBox(b));
            let cell_ref : Rc<RefCell<Shape>> = Rc::new(cell);
            tree.add_shape(i as usize, false, &cell_ref);
            i += 1;
        }

        tree.build();

        let mut res = Vec::new();

        tree.shapes_in_box(&BoundingBox {
            point_low: DVec2 {x: 5.5, y: 50.5},
            point_high: DVec2 {x: 6.5, y: 51.5}
        }, &mut |s| res.push((s.object_id, s.shape)));

        assert_eq!(res.len(), 4);

        let mut tmp = FxHashSet::default();

        tmp.insert(res[0].0);
        tmp.insert(res[1].0);
        tmp.insert(res[2].0);
        tmp.insert(res[3].0);

        assert!(tmp.contains(&550));
        assert!(tmp.contains(&551));
        assert!(tmp.contains(&650));
        assert!(tmp.contains(&651));
    }
}
