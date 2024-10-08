use std::cell::RefCell;
use std::f64::consts::PI;
use std::rc::Rc;
use glam::DVec2;
use rustc_hash::{FxHashMap, FxHashSet};
use crate::core::shapes::{Shape, SphereData};
use crate::core::shapes::Shape::Sphere;
use crate::core::tree::{BoundingBox, BoundingBoxTree, TreeSearchResult};

pub struct FixedBody {
    pub(crate) shapes: Vec<Rc<RefCell<Shape>>>,
}

#[derive(Clone)]
pub struct Ball {
    pub id: usize,
    pub shape: SphereData,
    pub mass: f64,
    pub velocity: DVec2,
}

#[derive(Clone)]
pub struct SystemState {
    pub balls: Vec<Ball>,
    pub time: f64
}

impl SystemState {
    pub fn len(&self) -> usize {
        2 * self.balls.len()
    }

    pub fn get(&self, idx: usize) -> (f64, f64) {
        let ball_num = idx / 2;
        let ball_f64_idx = idx % 2;

        let curr_ball = &self.balls[ball_num];

        match ball_f64_idx {
            0 => {
                (curr_ball.shape.pos.x, curr_ball.velocity.x)
            }
            _ => {
                (curr_ball.shape.pos.y, curr_ball.velocity.y)
            }
        }
    }

    pub fn get_mut(&mut self, idx: usize) -> (&mut f64, &mut f64) {
        let ball_num = idx / 2;
        let ball_f64_idx = idx % 2;

        let curr_ball = &mut self.balls[ball_num];

        match ball_f64_idx {
            0 => {
                (&mut curr_ball.shape.pos.x, &mut curr_ball.velocity.x)
            }
            _ => {
                (&mut curr_ball.shape.pos.y, &mut curr_ball.velocity.y)
            }
        }
    }
}

pub struct EvaluationResult {
    pub collision_present: bool,
    pub ball_accelerations: Vec<DVec2>
}

pub struct CollisionData {
    pub(crate) stiffness: f64,
    pub(crate) damping: f64
}

pub struct MaterialsData {
    pub(crate) ball_ball: CollisionData,
    pub(crate) ball_peg: CollisionData
}


struct BallCachedInfo {
    result_calculated_for: BoundingBox,
    result: Vec<TreeSearchResult>
}

#[derive(Clone)]
pub enum ExternalAccelerationMode {
    Disabled,
    Sinusoidal(DVec2, f64)
}

pub struct Environment {
    pub fixed_bodies: Vec<FixedBody>,
    tree: BoundingBoxTree,
    pub materials_data: MaterialsData,
    cached_ball_shapes: FxHashMap<usize, Rc<RefCell<Shape>>>,
    cached_ball_info: FxHashMap<usize, BallCachedInfo>,
    curr_external_acceleration: DVec2,
    external_acceleration: ExternalAccelerationMode
}

impl Environment {

    pub fn new(fixed_bodies: Vec<FixedBody>, materials_data: MaterialsData, external_acceleration: ExternalAccelerationMode) -> Environment {

        let mut tree = BoundingBoxTree::new();

        for (i, b) in fixed_bodies.iter().enumerate() {
            for sh in b.shapes.iter() {
                tree.add_shape(i, false, sh);
            }
        }

        Environment {
            fixed_bodies,
            tree,
            materials_data,
            cached_ball_shapes: FxHashMap::default(),
            cached_ball_info: FxHashMap::default(),
            curr_external_acceleration: DVec2::new(0.0, 0.0),
            external_acceleration
        }
    }

    fn check_cache(&mut self, ball: &Ball) -> bool {
        let cache_res = self.cached_ball_info.get(&ball.id);
        let bbox = Sphere(ball.shape.clone()).get_bounding_box();

        if let Some(cache_res) = cache_res {
            if !bbox.is_inside(&cache_res.result_calculated_for) {
                self.cached_ball_info.clear();
                return true;
            } else {
                return false;
            }
        }

        self.cached_ball_info.clear();
        true
    }

    fn insert_into_cache(&mut self, ball: &Ball) {
        let cache_res = self.cached_ball_info.get(&ball.id);

        if cache_res.is_some() {
            return;
        }

        let shape = Sphere(ball.shape.clone());
        let bbox = shape.get_bounding_box();


        let bbox = bbox.extended(1.1);

        let mut res = Vec::new();

        self.tree.shapes_in_box(&bbox, &mut |x| res.push(x));

        self.cached_ball_info.insert(ball.id, BallCachedInfo {
            result_calculated_for: bbox,
            result: res
        });
    }

    fn is_ball(&self, id: usize) -> bool {
        self.fixed_bodies.len() < id
    }

    fn evaluate_ball(&mut self, ball: &Ball) -> (DVec2, bool) {

        let ball_shape = Shape::Sphere(ball.shape.clone());

        let mut res = DVec2{x:0.0,y:0.0};

        res.y -= 9.81;

        res += self.curr_external_acceleration;

       // println!("{} {}", res.x, res.y);

        let mut collision = false;

        self.insert_into_cache(ball);

        for tres in self.cached_ball_info.get(&ball.id).unwrap().result.iter() {
            if tres.object_id == ball.id {
                continue;
            }

            if let Some(intersection) = tres.shape.borrow().get_intersection(&ball_shape) {

                collision = true;

                let collision_properties;

                if self.is_ball(tres.object_id) {
                    collision_properties = &self.materials_data.ball_ball;
                } else {
                    collision_properties = &self.materials_data.ball_peg;
                }

                let ball_velocity_proj = ball.velocity.project_onto_normalized(intersection.normal);

                let mut ball_force = DVec2 {x: 0.0, y: 0.0};
                ball_force += intersection.area * collision_properties.stiffness * intersection.normal / ball.mass;
                assert!(!f64::is_nan(ball_force.x) || !f64::is_nan(ball_force.y));
                ball_force += f64::sqrt(intersection.area) * collision_properties.damping * (-ball_velocity_proj) / ball.mass;
                assert!(!f64::is_nan(ball_force.x) && !f64::is_nan(ball_force.y));

                res += ball_force;
            }
        }

        if f64::is_nan(res.x) || f64::is_nan(res.y) {
            panic!();
        }

        (res, collision)
    }

    pub fn evaluate_state(&mut self, state: &SystemState) -> EvaluationResult {

        self.curr_external_acceleration = match self.external_acceleration {
            ExternalAccelerationMode::Disabled => { DVec2::new(0.0, 0.0) }
            ExternalAccelerationMode::Sinusoidal(amp, freq) => { DVec2::new( f64::sin(state.time * freq *2.0*PI) * amp.x, f64::sin(state.time * freq *2.0*PI) * amp.y ) }
        };

        for b in state.balls.iter() {
            let cache_res = self.cached_ball_shapes.get(&b.id);
            let shape = Sphere(b.shape.clone());

            if let Some(cache_res) = cache_res {
                *cache_res.borrow_mut() = shape;
            } else {
                let rc = Rc::new(RefCell::new(shape));
                self.tree.add_shape(b.id, true, &rc);
                self.cached_ball_shapes.insert(b.id, rc);
            }
        }

        let mut tree_needs_update = false;

        for b in state.balls.iter() {
            if self.check_cache(b) {
                tree_needs_update = true;
                break;
            }
        }

        if self.cached_ball_shapes.len() > state.balls.len() {
            let mut ids_to_delete: FxHashSet<usize> = self.cached_ball_shapes.keys().map(|x| *x).collect();

            for b in state.balls.iter() {
                ids_to_delete.remove(&b.id);
            }

            for id in ids_to_delete.into_iter() {
                self.cached_ball_shapes.remove(&id);
                self.tree.delete(id);
            }
        }

        if tree_needs_update {
            self.tree.update();
        }

        let mut ball_accelerations = Vec::new();

        let mut collision_present = false;

        for i in 0..state.balls.len() {
            let (acc, collision) = self.evaluate_ball(&state.balls[i]);
            collision_present |= collision;
            ball_accelerations.push(acc);
        }

        EvaluationResult {
            collision_present,
            ball_accelerations
        }
    }
}


#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::rc::Rc;
    use glam::DVec2;
    use crate::core::environment::{Ball, CollisionData, Environment, ExternalAccelerationMode, FixedBody, MaterialsData, SystemState};
    use crate::core::shapes::{Shape, SphereData, TriangleData};

    fn get_test_materials() -> MaterialsData {
        MaterialsData {
            ball_ball: CollisionData{ stiffness: 1e9, damping: 1e9},
            ball_peg: CollisionData{stiffness: 5e8, damping: 1e9}
        }
    }

    #[test]
    fn environment_1() {
        let mut env = Environment::new(vec![
            FixedBody {
                shapes: vec![Rc::new(RefCell::new(Shape::Triangle(
                    TriangleData {
                        vertices: [
                            DVec2 {x: 0.0, y: -2.0},
                            DVec2 {x: 2.0, y: 0.0},
                            DVec2 {x: 2.0, y: -2.0}
                        ]
                    }
                )))],
            },
            FixedBody {
                shapes: vec![Rc::new(RefCell::new(Shape::Sphere(
                    SphereData {
                        r: std::f64::consts::SQRT_2,
                        pos: DVec2{x: -2.0, y: -2.0}
                    }
                )))],
            }
        ], get_test_materials(), ExternalAccelerationMode::Disabled);

        let state = SystemState {
            time: 0.0,
            balls: vec![
                Ball {
                    id: 2,
                    shape: SphereData {
                        r: std::f64::consts::SQRT_2,
                        pos: DVec2{x: 0.0, y: -0.00001}
                    },
                    mass: 1.0,
                    velocity: DVec2{x: 0.0, y: 0.0},
                }
            ]
        };

        let res = env.evaluate_state(&state);

        assert_eq!(res.collision_present, true);

        assert_eq!(res.ball_accelerations.len(), 1);

        assert!((res.ball_accelerations[0].x - (-4.3661633662807233)).abs() < f64::EPSILON);
        assert!((res.ball_accelerations[0].y - 15.637976522631293).abs() < f64::EPSILON);
    }
}
