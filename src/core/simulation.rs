use std::cell::RefCell;
use crate::core::environment::{Environment, SystemState};

#[derive(PartialEq)]
enum TimeStepState {
    NoCollision,
    CollisionHappening,
    AwaitingCollision,
    ApproachingCollision
}

pub struct SimulationRunner {
    environment: Environment,
    curr_state: Option<SystemState>,
    history: Vec<SystemState>,
    time_step_state: TimeStepState
}

impl SimulationRunner {
    const STEP_SHORT: f64 = 1e-6;
    const STEP_LONG: f64 = 1e-3;
    const STEP_MEDIUM: f64 = 3.3e-5;

    pub fn new(environment: Environment) -> SimulationRunner {
        SimulationRunner {
            environment,
            curr_state: None,
            history: Vec::new(),
            time_step_state: TimeStepState::NoCollision
        }
    }

    pub fn set_state(&mut self, state: SystemState) {
        self.curr_state = Some(state);
        self.history.clear();
        self.history.push( self.curr_state.as_ref().unwrap().clone());
    }

    pub fn run(&mut self, time: f64) {
        let mut curr_time;

        match self.history.last() {
            None => { curr_time = 0.0 }
            Some(s) => { curr_time = s.time }
        }

        let destination_time = time + curr_time;

        let collision_present = RefCell::new(false);

        let mut diff_eq_func = |s: &SystemState| -> Vec<f64> {

            let ev_res = self.environment.evaluate_state(s);

            if ev_res.collision_present {
                *collision_present.borrow_mut() = true;
            }

            let mut res = Vec::new();

            for a in ev_res.ball_accelerations.iter() {
                res.push(a.x);
                res.push(a.y);
            }

            res
        };

        while curr_time < destination_time {
            let time_step = match self.time_step_state {
                TimeStepState::NoCollision => {
                    SimulationRunner::STEP_LONG
                }
                TimeStepState::CollisionHappening => {
                    SimulationRunner::STEP_SHORT
                }
                TimeStepState::AwaitingCollision => {
                    SimulationRunner::STEP_SHORT
                }
                TimeStepState::ApproachingCollision => {
                    SimulationRunner::STEP_MEDIUM
                }
            };

            *collision_present.borrow_mut() = false;
            let next_state = rkn4_step(time_step, self.curr_state.as_ref().unwrap(), &mut diff_eq_func);

            // If collision happened at long time step, we need to roll back one step and do it again with short time step
            if self.time_step_state == TimeStepState::NoCollision && *collision_present.borrow() {
                self.time_step_state = TimeStepState::ApproachingCollision;
                continue;
            }
            else if self.time_step_state == TimeStepState::ApproachingCollision && *collision_present.borrow() {
                self.time_step_state = TimeStepState::AwaitingCollision;
                continue;
            }
            else if self.time_step_state == TimeStepState::AwaitingCollision && *collision_present.borrow() {
                self.time_step_state = TimeStepState::CollisionHappening;
            }
            else if self.time_step_state == TimeStepState::CollisionHappening && !*collision_present.borrow() {
                self.time_step_state = TimeStepState::NoCollision;
            }

            curr_time += time_step;

            //println!("Time {} Step {} Ball.x {} Ball.y {}", curr_time, time_step, next_state.balls[0].shape.pos.x, next_state.balls[0].shape.pos.y);

            self.curr_state = Some(next_state);

            let next_frame_t = self.history.last().unwrap().time + 1e-3;
            if curr_time > next_frame_t {
                self.history.push(self.curr_state.as_ref().unwrap().clone());
            }
        }
    }

    pub fn get_history(&self) -> &[SystemState] {
        &self.history
    }
}


fn set_state_like(state: &mut SystemState, x: &[f64], x_dot: &[f64]) {
    let n = state.len();
    assert_eq!(x.len(), n);
    assert_eq!(x_dot.len(), n);

    for i in 0..n {
        let (a,b) = state.get_mut(i);
        *a = x[i];
        *b = x_dot[i];
    }
}

fn rkn4_step<F>(step: f64, state: &SystemState, mut f: F) -> SystemState
    where
        F: FnMut(&SystemState) -> Vec<f64>
{

    let n = state.len();

    let k1 = f(state);

    let mut tmp_state = state.clone();

    assert_eq!(k1.len(), n);

    let mut x1_dot = vec![0.0; n];
    let mut x1 = vec![0.0; n];

    for i in 0..n {
        let (x, x_dot) = state.get(i);
        x1_dot[i] = x_dot + k1[i] * (step / 2.0);
        x1[i] = x + (1.0/6.0) * (step / 2.0) * ((step / 2.0) * k1[i] + 4.0 * x_dot + 2.0 * x1_dot[i]);
    }

    set_state_like(&mut tmp_state, &x1, &x1_dot);
    let k2 = f(&tmp_state);

    let mut x2_dot = vec![0.0; n];
    let mut x2 = vec![0.0; n];

    for i in 0..n {
        let (x, x_dot) = state.get(i);
        x2_dot[i] = x_dot + k2[i] * (step / 2.0);
        x2[i] = x + (1.0/6.0) * (step/2.0) * ((step/2.0)*k1[i] + 4.0*x_dot + 2.0*x2_dot[i]);
    }

    set_state_like(&mut tmp_state, &x2, &x2_dot);
    let k3 = f(&tmp_state);

    let mut x3_dot = vec![0.0; n];
    let mut x3 = vec![0.0; n];

    for i in 0..n {
        let (x, x_dot) = state.get(i);
        x3_dot[i] = x_dot + k2[i] * step;
        x3[i] = x + (1.0/6.0)*step * (step * k1[i] + 4.0*x_dot + 2.0*x3_dot[i]);
    }

    set_state_like(&mut tmp_state, &x3, &x3_dot);
    let k4 = f(&tmp_state);

    let mut next_x = vec![0.0; n];
    let mut next_x_dot = vec![0.0; n];

    for i in 0..n {
        let (x, x_dot) = state.get(i);
        next_x_dot[i] = x_dot + (step / 6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
        next_x[i] = x + (step / 6.0) * (x_dot + 2.0*x1_dot[i] + 2.0*x2_dot[i] + x3_dot[i]);
    }

    set_state_like(&mut tmp_state, &next_x, &next_x_dot);
    tmp_state.time = state.time + step;
    tmp_state
}

/*
#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::collections::HashMap;
    use std::rc::Rc;
    use glam::DVec2;
    use crate::core::environment::{Ball, CollisionData, Environment, FixedBody, MaterialsData, SystemState};
    use crate::core::shapes::{Shape, SphereData, TriangleData};
    use crate::core::simulation::{rkn4_step, SimulationRunner};

    #[test]
    fn rkn_square() {
        let f = |x: &Vec<f64>, x_dot: &Vec<f64>| -> Vec<f64> {

            assert_eq!(x.len(), 1);
            assert_eq!(x_dot.len(), 1);

            vec![2.0]
        };

        let mut t = 0.0;
        let mut curr_x = 0.0;
        let mut curr_x_dot = 0.0;

        let step = 1e-3;

        for _ in 0..10000 {
            let (res, res_dot) = rkn4_step(step, vec![curr_x], vec![curr_x_dot], f);
            curr_x = res[0];
            curr_x_dot = res_dot[0];
            t += step;

            assert!((curr_x - t*t).abs() < 1e-10);
        }
    }

    #[test]
    fn rkn_exp() {
        let f = |x: &Vec<f64>, x_dot: &Vec<f64>| -> Vec<f64> {

            assert_eq!(x.len(), 1);
            assert_eq!(x_dot.len(), 1);

            vec![x[0]]
        };

        let mut t = 0.0;
        let mut curr_x = 1.0;
        let mut curr_x_dot = 1.0;

        let step = 1e-3;

        for _ in 0..10000 {
            let (res, res_dot) = rkn4_step(step, vec![curr_x], vec![curr_x_dot], f);
            curr_x = res[0];
            curr_x_dot = res_dot[0];
            t += step;

            assert!((curr_x - f64::exp(t)).abs() < 1e-8);
        }

        println!("{}", curr_x);
    }

    #[test]
    fn rkn_harmonic_with_damping() {
        let f = |x: &Vec<f64>, x_dot: &Vec<f64>| -> Vec<f64> {

            assert_eq!(x.len(), 1);
            assert_eq!(x_dot.len(), 1);

            vec![-x[0] - 0.1*x_dot[0]]
        };

        let mut t = 0.0;
        let mut curr_x = 0.0;
        let mut curr_x_dot = 1.0;

        let step = 1e-3;

        let sqrt399 = f64::sqrt(399.0);

        let A = 20.0 / sqrt399;

        for _ in 0..10000 {
            let (res, res_dot) = rkn4_step(step, vec![curr_x], vec![curr_x_dot], f);
            curr_x = res[0];
            curr_x_dot = res_dot[0];
            t += step;

            assert!((curr_x - A*f64::exp(-t/20.0)*f64::sin(sqrt399*t/20.0)).abs() < 1e-10);
        }

    }

    fn get_test_materials() -> MaterialsData {
        let mut materials = HashMap::new();
        materials.insert((0,0), (CollisionData{ stiffness: 1e6, damping: 0.0}, FrictionData{}));
        materials.insert((0,1), (CollisionData{ stiffness: 5e6, damping: 1e7}, FrictionData{}));

        MaterialsData {
            data: materials
        }
    }

    #[test]
    fn simulation_ball_free_fall() {
        let env = Environment::new(vec![], get_test_materials());

        let mut sim = SimulationRunner::new(env);

        let initial = SystemState {
            time: 0.0,
            balls: vec![
                Ball {
                    shape: SphereData {
                        pos: DVec2{x: 0.0, y:0.0},
                        r: 1.0
                    },
                    rot: 0.0,
                    mass: 2.5,
                    velocity: DVec2{x:0.0,y:0.0},
                    angular_velocity: 0.0,
                    material_id: 0
                }
            ]
        };

        sim.set_state(initial);

        sim.run(2.0);

        let res = sim.get_history();

        for (t, state) in res.iter() {
            let t = *t;
            let pos = -state.balls[0].shape.pos.y;

            assert!((pos - 0.5*9.81*t*t).abs() < 1e-10);
        }
    }

    #[test]
    fn simulation_ball_bounce() {
        let env = Environment::new(vec![
            FixedBody {
                shapes: vec![
                    Rc::new(RefCell::new(Shape::Triangle(TriangleData {
                        vertices: [
                            DVec2 {x: -1.0, y: 0.0},
                            DVec2 {x: 1.0, y: 0.0},
                            DVec2 {x: 0.0, y: -1.0}
                        ]
                    })))
                ],
                material_id: 0
            }
        ], get_test_materials());

        let mut sim = SimulationRunner::new(env);

        let initial = SystemState {
            time: 0.0,
            balls: vec![
                Ball {
                    shape: SphereData {
                        pos: DVec2{x: 0.0, y:5.0},
                        r: 1.0
                    },
                    rot: 0.0,
                    mass: 2.5,
                    velocity: DVec2{x:0.0,y:0.0},
                    angular_velocity: 0.0,
                    material_id: 0
                }
            ]
        };

        sim.set_state(initial);

        sim.run(2.0);

        let res = sim.get_history();

        let mut highest_after_bounce = 0.0;

        let mut bounce_done = false;

        for (t, state) in res.iter() {
            println!("{}", state.balls[0].shape.pos.y);
            if state.balls[0].shape.pos.y < 1.0 {
                bounce_done = true;
            }

            if bounce_done {
                if state.balls[0].shape.pos.y > highest_after_bounce {
                    highest_after_bounce = state.balls[0].shape.pos.y;
                }
            }
        }

        assert!((highest_after_bounce - 5.0).abs() < 1e-3);
    }
}


*/