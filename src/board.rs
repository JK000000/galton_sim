use std::cell::RefCell;
use std::cmp::min;
use std::f64::consts::PI;
use std::ops::Deref;
use std::rc::Rc;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;
use glam::DVec2;
use rand::Rng;
use rustc_hash::{FxHashSet};
use crate::core::environment::{Ball, CollisionData, Environment, ExternalAccelerationMode, FixedBody, MaterialsData, SystemState};
use crate::core::shapes::{Shape, SphereData, TriangleData};
use crate::core::shapes::Shape::Triangle;
use crate::core::simulation::SimulationRunner;
use crate::core::tree::BoundingBox;
use crate::io::output::ProgressPrinter;

#[derive(Clone)]
pub enum PegType {
    Circle,
    Triangle,
}

#[derive(Clone)]
pub enum PegSpacing {
    Regular(f64),
    WiderOnTop(f64),
}

#[derive(Clone)]
pub struct BoardMaterialsData {
    pub(crate) ball_peg_stiffness: f64,
    pub(crate) ball_peg_damping: f64,
    pub(crate) ball_ball_stiffness: f64,
    pub(crate) ball_ball_damping: f64,
}

#[derive(Clone)]
pub struct BallRunInfo {
    pub row_exit_pos: Vec<i32>,
    pub initial_offset_x: f64,
    pub initial_offset_y: f64,
}

#[derive(Clone)]
pub struct Board {
    pub(crate) peg_type: PegType,
    pub(crate) peg_spacing: PegSpacing,
    pub(crate) row_spacing: f64,
    pub(crate) peg_size: f64,
    pub(crate) ball_size: f64,
    pub(crate) ball_density: f64,
    pub(crate) num_rows: u32,
    pub(crate) materials_data: BoardMaterialsData,
    pub(crate) shaking_mode: ExternalAccelerationMode
}

fn generate_funnel_simple<T>(exit_y: f64, exit_diameter: f64, angle: f64, num_ball_rows: u32, ball_diameter: f64, ball_mass: f64, mut ball_pos_perturbation: T) -> (Vec<Shape>, Vec<Ball>)
    where T: FnMut() -> f64
{
    let ball_relative_distance = 1.05;

    let h = (num_ball_rows as f64) * ball_diameter * ball_relative_distance * f64::sqrt(3.0) / 2.0 + 0.005;

    let mut p1 = DVec2 { x: exit_diameter / 2.0, y: exit_y };
    let mut p2 = DVec2 { x: exit_diameter / 2.0, y: exit_y + 0.001 };
    let mut p3 = DVec2 { x: exit_diameter / 2.0 + h * f64::tan(angle), y: exit_y + 0.001 + h };
    let mut p4 = DVec2 { x: exit_diameter / 2.0 + h * f64::tan(angle), y: exit_y + h };

    let t1 = Triangle(TriangleData { vertices: [p1.clone(), p2.clone(), p3.clone()] });
    let t2 = Triangle(TriangleData { vertices: [p1.clone(), p3.clone(), p4.clone()] });

    p1.x = -p1.x;
    p2.x = -p2.x;
    p3.x = -p3.x;
    p4.x = -p4.x;

    let t3 = Triangle(TriangleData { vertices: [p1.clone(), p2.clone(), p3.clone()] });
    let t4 = Triangle(TriangleData { vertices: [p1.clone(), p3.clone(), p4.clone()] });

    let mut balls = Vec::new();

    let mut next_ball_id = 1000000;

    for i in 0..num_ball_rows {
        let row_y = exit_y + ((i as f64) + 0.5) * ball_diameter * ball_relative_distance * f64::sqrt(3.0) / 2.0;
        let ball_max_x = (row_y - 0.001 - exit_y) * f64::tan(angle) + ball_diameter / 2.0 - ball_diameter * f64::tan(angle) / (2.0 * f64::cos(angle));

        let mut ball_pos_x;

        if i % 2 == 0 {
            balls.push(Ball {
                id: next_ball_id,
                shape: SphereData {
                    r: ball_diameter / 2.0,
                    pos: DVec2 { x: 0.0 + ball_pos_perturbation(), y: row_y + ball_pos_perturbation() },
                },
                mass: ball_mass,
                velocity: DVec2 { x: 0.0, y: 0.0 },
            });
            next_ball_id += 1;

            ball_pos_x = ball_diameter * ball_relative_distance;
        } else {
            ball_pos_x = ball_diameter * ball_relative_distance * 0.5;
        }

        while ball_pos_x < ball_max_x {
            balls.push(Ball {
                id: next_ball_id,
                shape: SphereData {
                    r: ball_diameter / 2.0,
                    pos: DVec2 { x: ball_pos_x + ball_pos_perturbation(), y: row_y + ball_pos_perturbation() },
                },
                mass: ball_mass,
                velocity: DVec2 { x: 0.0, y: 0.0 },
            });
            next_ball_id += 1;

            balls.push(Ball {
                id: next_ball_id,
                shape: SphereData {
                    r: ball_diameter / 2.0,
                    pos: DVec2 { x: -ball_pos_x + ball_pos_perturbation(), y: row_y + ball_pos_perturbation() },
                },
                mass: ball_mass,
                velocity: DVec2 { x: 0.0, y: 0.0 },
            });
            next_ball_id += 1;

            ball_pos_x += ball_diameter * ball_relative_distance;
        }
    }

    (vec![t1, t2, t3, t4], balls)
}

pub enum FunnelType {
    Simple(f64)
}

const MAX_SUPPORTED_X: i32 = 120;
const MAX_TIME_PER_BALL: f64 = 2.0;

impl Board {
    fn get_materials(&self) -> MaterialsData {
        MaterialsData {
            ball_ball: CollisionData { stiffness: self.materials_data.ball_ball_stiffness, damping: self.materials_data.ball_ball_damping },
            ball_peg: CollisionData { stiffness: self.materials_data.ball_peg_stiffness, damping: self.materials_data.ball_peg_damping },
        }
    }

    fn get_peg_pos_x(&self, row_idx: u32, ped_idx: i32) -> f64 {
        match &self.peg_spacing {
            PegSpacing::Regular(mult) => {
                let mult = *mult;
                (ped_idx as f64) * (mult * 2.0 * self.peg_size) -
                    if row_idx % 2 == 0 { 0.0 } else { mult * self.peg_size }
            }
            PegSpacing::WiderOnTop(ratio) => {
                let ratio = *ratio;
                let space = (2.0 * self.peg_size) * (row_idx as f64 / (self.num_rows - 1) as f64) + (2.0 * self.peg_size * ratio) * ((self.num_rows - 1 - row_idx) as f64 / (self.num_rows - 1) as f64);

                (ped_idx as f64) * (space) -
                    if row_idx % 2 == 0 { 0.0 } else { space / 2.0 }
            }
        }
    }

    fn get_row_y(&self, row_idx: u32) -> f64 {
        -(row_idx as f64) * self.row_spacing * self.peg_size
    }

    fn get_environment_bodies(&self) -> Vec<FixedBody> {
        let mut fixed_bodies = Vec::new();

        for i in 0..self.num_rows {
            for j in -MAX_SUPPORTED_X..MAX_SUPPORTED_X {
                let pos_x = self.get_peg_pos_x(i, j);

                let pos_y = self.get_row_y(i);

                let shape = match self.peg_type {
                    PegType::Circle => {
                        Shape::Sphere(SphereData {
                            r: self.peg_size / 2.0,
                            pos: DVec2 { x: pos_x, y: pos_y },
                        })
                    }
                    PegType::Triangle => {
                        Shape::Triangle(TriangleData {
                            vertices: [
                                DVec2 { x: pos_x, y: pos_y + self.peg_size * f64::sqrt(3.0) / 3.0 },
                                DVec2 { x: pos_x - self.peg_size / 2.0, y: pos_y - self.peg_size * f64::sqrt(3.0) / 6.0 },
                                DVec2 { x: pos_x + self.peg_size / 2.0, y: pos_y - self.peg_size * f64::sqrt(3.0) / 6.0 }
                            ]
                        })
                    }
                };

                fixed_bodies.push(FixedBody {
                    shapes: vec![
                        Rc::new(RefCell::new(shape))
                    ],
                })
            }
        }

        fixed_bodies
    }

    pub fn get_environment(&self) -> Environment {
        let fixed_bodies = self.get_environment_bodies();

        Environment::new(fixed_bodies, self.get_materials(), self.shaking_mode.clone())
    }

    pub fn get_environment_and_initial_state_with_funnel(&self, funnel_type: &FunnelType, num_rows: u32) -> (Environment, SystemState) {
        let mut fixed_bodies = self.get_environment_bodies();

        let ball_mass = self.ball_density * (4.0 / 3.0) * (self.ball_size / 2.0).powi(3);

        let mut rng = rand::thread_rng();

        let m_balls;

        match funnel_type {
            FunnelType::Simple(angle) => {
                let (fixed, balls) = generate_funnel_simple(self.peg_size * self.row_spacing, 3.0 * self.ball_size, *angle, num_rows, self.ball_size, ball_mass,
                                                            || {
                                                                let mut res: f64 = rng.gen();
                                                                res -= 0.5;
                                                                res *= 2e-5;
                                                                res
                                                            });

                let funnel_body = FixedBody {
                    shapes: fixed.into_iter().map(|s| Rc::new(RefCell::new(s))).collect(),
                };

                fixed_bodies.push(funnel_body);
                m_balls = balls;
            }
        }

        let initial_state = SystemState {
            balls: m_balls,
            time: 0.0,
        };

        (Environment::new(fixed_bodies, self.get_materials(),self.shaking_mode.clone()), initial_state)
    }

    fn get_ball_slot_idx(&self, ball_pos: &DVec2, curr_row_idx: u32) -> i32 {
        let ball_x = ball_pos.x;

        if ball_x > self.get_peg_pos_x(curr_row_idx, 0) {
            for i in 1..(MAX_SUPPORTED_X - 1) {
                if ball_x < self.get_peg_pos_x(curr_row_idx, i) {
                    return i - 1;
                }
            }
        } else {
            for i in 1..(MAX_SUPPORTED_X - 1) {
                if ball_x > self.get_peg_pos_x(curr_row_idx, -i) {
                    return -i;
                }
            }
        }

        return 2137;

        //panic!();
    }

    pub fn run_single_ball_with_initial_conditions(&self, start_x: f64, start_y: f64, start_v_x: f64, start_v_y: f64) -> (BallRunInfo, Vec<SystemState>) {
        let env = self.get_environment();

        let mut sim = SimulationRunner::new(env);

        let initial = SystemState {
            time: 0.0,
            balls: vec![
                Ball {
                    id: 1000000,
                    shape: SphereData {
                        pos: DVec2 { x: 0.0 + start_x, y: self.row_spacing * self.peg_size + start_y },
                        r: self.ball_size / 2.0,
                    },
                    mass: self.ball_density * (4.0 / 3.0) * PI * (self.ball_size / 2.0) * (self.ball_size / 2.0) * (self.ball_size / 2.0),
                    velocity: DVec2 { x: start_v_x, y: start_v_y },
                }
            ],
        };

        sim.set_state(initial);

        let detection_border = self.peg_size * self.row_spacing * self.num_rows as f64 + self.peg_size * 0.5;



        loop {
            sim.run(0.2);

            let last_state = sim.get_history().last().unwrap();

            if last_state.time > MAX_TIME_PER_BALL {
                eprintln!("MAX_TIME_PER_BALL exceeded");
                eprintln!("Position {} {}", last_state.balls[0].shape.pos.x,last_state.balls[0].shape.pos.y );
                break;
            }

            let ball_y = last_state.balls[0].shape.pos.y;

            if ball_y < -detection_border {
                break;
            }
        }

        let res = Vec::from(sim.get_history());


        let mut row_exit_pos = Vec::new();

        for state in res.iter() {
            let curr_row_idx = row_exit_pos.len() as u32;

            if curr_row_idx >= self.num_rows {
                break;
            }

            if state.balls[0].shape.pos.y < self.get_row_y(curr_row_idx) {
                row_exit_pos.push(self.get_ball_slot_idx(&state.balls[0].shape.pos, curr_row_idx));
            }
        }

        (BallRunInfo {
            row_exit_pos,
            initial_offset_x: start_x,
            initial_offset_y: start_y,
        }, res)
    }

    pub fn run_single_ball(&self) -> (BallRunInfo, Vec<SystemState>) {

        let mut rng = rand::thread_rng();

        let mut start_x: f64 = rng.gen();
        let mut start_y: f64 = rng.gen();

        start_x -= 0.5;
        start_y -= 0.5;

        start_x *= 1e-3;
        start_y *= 1e-3;

        self.run_single_ball_with_initial_conditions(start_x, start_y, 0.0, 0.0)
    }

    const BALL_RUN_INFOS_IDX_OFFSET: usize = 1000000;

    fn insert_balls(&self, state: &mut SystemState, ball_run_infos: &mut Vec<BallRunInfo>, max_num_balls: u32) {
        let allowed_area_center = DVec2::new(0.0, self.peg_size * self.row_spacing);

        let allowed_area = BoundingBox {
            point_low: allowed_area_center + DVec2::new(-0.5*self.peg_size, -0.5*self.peg_size),
            point_high: allowed_area_center + DVec2::new(0.5*self.peg_size, 0.5*self.peg_size),
        };

        let mut covered = 0.0;

        let mut ball_shapes = Vec::new();

        for b in state.balls.iter() {
            let m_shape = Shape::Sphere(b.shape.clone());
            let common_area = BoundingBox::intersect(&m_shape.get_bounding_box(), &allowed_area);
            if let Some(common_area) = common_area {
                ball_shapes.push(m_shape);
                covered += common_area.surface_area();
            }
        }

        let ratio_taken = covered / allowed_area.surface_area();

        if ratio_taken < 0.5 {
            let mut balls_left = max_num_balls;

            let mut rng = rand::thread_rng();

            for _ in 0..10 {
                if balls_left == 0 {
                    break;
                }

                let ball_center = DVec2::new(rng.gen_range(allowed_area.point_low.x..allowed_area.point_high.x), rng.gen_range(allowed_area.point_low.y..allowed_area.point_high.y));

                let new_ball_shape = Shape::Sphere(SphereData {
                    r: self.ball_size,
                    pos: ball_center,
                });

                let mut ok = true;

                for s in ball_shapes.iter() {
                    if s.get_intersection(&new_ball_shape).is_some() {
                        ok = false;
                        break;
                    }
                }

                if ok {
                    state.balls.push(Ball {
                        id: Board::BALL_RUN_INFOS_IDX_OFFSET + ball_run_infos.len(),
                        shape: SphereData {
                            r: self.ball_size / 2.0,
                            pos: ball_center,
                        },
                        mass: self.ball_density * (4.0 / 3.0) * PI * (self.ball_size / 2.0) * (self.ball_size / 2.0) * (self.ball_size / 2.0),
                        velocity: DVec2 { x: 0.0, y: 0.0 },
                    });
                    ball_run_infos.push(BallRunInfo {
                        row_exit_pos: Vec::new(),
                        initial_offset_x: ball_center.x - allowed_area_center.x,
                        initial_offset_y: ball_center.y - allowed_area_center.y,
                    });

                    ball_shapes.push(new_ball_shape);
                    balls_left -= 1;
                }
            }
        }
    }

    fn process_balls(&self, state: &mut SystemState, ball_run_infos: &mut Vec<BallRunInfo>, history: &[SystemState], progress: &Arc<Mutex<ProgressPrinter>>) {
        let mut deleted_idx = FxHashSet::default();

        for s in history {
            for (i, b) in s.balls.iter().enumerate() {
                let curr_row_idx = ball_run_infos[b.id - Board::BALL_RUN_INFOS_IDX_OFFSET].row_exit_pos.len() as u32;

                if curr_row_idx >= self.num_rows {
                    if !deleted_idx.contains(&i) {
                        panic!();
                    }
                    continue;
                }

                let target_y = self.get_row_y(curr_row_idx);

                if b.shape.pos.y < target_y {
                    ball_run_infos[b.id - Board::BALL_RUN_INFOS_IDX_OFFSET].row_exit_pos.push(self.get_ball_slot_idx(&b.shape.pos, curr_row_idx));

                    if curr_row_idx + 1 == self.num_rows {
                        deleted_idx.insert(i);
                    }
                }
            }
        }

        for _ in 0..deleted_idx.len() {
            progress.lock().unwrap().increase();
        }

        let mut new_state = Vec::new();

        for i in 0..state.balls.len() {
            if deleted_idx.contains(&i) {
                continue;
            }

            new_state.push(state.balls[i].clone());
        }

        state.balls.clear();

        for i in 0..new_state.len() {
            state.balls.push(new_state[i].clone());
        }
    }

    pub fn run_many_balls(&self, num_balls: u32, progress: &Option<Arc<Mutex<ProgressPrinter>>>) -> (Vec<BallRunInfo>, Vec<SystemState>) {
        let env = self.get_environment();

        let m_progress_owned;

        let m_progress;

        if progress.is_none() {
            m_progress_owned = Arc::new(Mutex::new(ProgressPrinter::new(num_balls)));
            m_progress = &m_progress_owned;
        } else {
            m_progress = progress.as_ref().unwrap();
        }

        let mut initial = SystemState {
            balls: Vec::new(),
            time: 0.0,
        };

        let mut ball_run_infos = Vec::new();

        self.insert_balls(&mut initial, &mut ball_run_infos, min(5, num_balls));

        let mut sim = SimulationRunner::new(env);

        sim.set_state(initial);

        let mut state;

        let mut history = Vec::new();

        let mut total_time = 0.0;

        loop {
            sim.run(0.03);

            let res = sim.get_history();

            history.extend(res.iter().map(|x| x.clone()));

            state = res.last().unwrap().clone();

            self.process_balls(&mut state, &mut ball_run_infos, res, m_progress);

            let balls_left = num_balls - ball_run_infos.len() as u32;

            //println!("BallsLeft {} State {} BallRunInfos {}", balls_left, state.balls.len(), ball_run_infos.len());

            if balls_left >= 5 {
                self.insert_balls(&mut state, &mut ball_run_infos, 5);
            } else if balls_left == 0 && state.balls.len() == 0 {
                break;
            } else if balls_left > 0 {
                self.insert_balls(&mut state, &mut ball_run_infos, balls_left);
            }

            sim.set_state(state.clone());
            total_time += 0.03;

            if total_time > num_balls as f64 * 0.08 + 1.0 {
                break;
            }
        }

        (ball_run_infos, history)
    }

    pub fn run_many_balls_with_funnel(&self, num_rows: u32, funnel_type: &FunnelType) -> (Vec<BallRunInfo>, Vec<SystemState>) {
        let (env, initial) = self.get_environment_and_initial_state_with_funnel(funnel_type, num_rows);

        let mut sim = SimulationRunner::new(env);

        sim.set_state(initial);

        let detection_border = self.peg_size * self.row_spacing * self.num_rows as f64 + self.peg_size * 0.5;

        loop {
            sim.run(0.4);

            let last_state = sim.get_history().last().unwrap();

            let mut exit = true;

            for b in last_state.balls.iter() {
                if b.shape.pos.y > -detection_border {
                    exit = false;
                    break;
                }
            }

            if exit {
                break;
            }
        }

        let res = Vec::from(sim.get_history());

        let mut run_infos = vec![BallRunInfo { row_exit_pos: Vec::new(), initial_offset_x: 0.0, initial_offset_y: 0.0 }; res[0].balls.len()];


        for state in res.iter() {
            for i in 0..state.balls.len() {
                let row_exit_pos = &mut run_infos[i].row_exit_pos;

                let curr_row_idx = row_exit_pos.len() as u32;

                if curr_row_idx >= self.num_rows {
                    continue;
                }

                if state.balls[i].shape.pos.y < self.get_row_y(curr_row_idx) {
                    let ball_x = state.balls[i].shape.pos.x;

                    if ball_x > self.get_peg_pos_x(curr_row_idx, 0) {
                        for i in 1..50 {
                            if ball_x < self.get_peg_pos_x(curr_row_idx, i) {
                                row_exit_pos.push(i - 1);
                                break;
                            }
                        }
                    } else {
                        for i in 1..50 {
                            if ball_x > self.get_peg_pos_x(curr_row_idx, -i) {
                                row_exit_pos.push(-i);
                                break;
                            }
                        }
                    }
                }
            }
        }

        (run_infos, res)
    }

    pub fn run_many_balls_many_iterations(&self, iterations_per_thread: u32, num_threads: u32, num_balls_per_run: u32) -> Vec<BallRunInfo> {
        let results: Arc<Mutex<Vec<BallRunInfo>>> = Arc::new(Mutex::new(Vec::new()));
        let m_board = Arc::new(Mutex::new(self.clone()));
        let progress = Arc::new(Mutex::new(ProgressPrinter::new(iterations_per_thread * num_threads * num_balls_per_run)));


        let mut handles = Vec::new();

        for _ in 0..num_threads {
            let results = Arc::clone(&results);
            let m_progress = Arc::clone(&progress);

            let m_board = Arc::clone(&m_board);

            let m_progress = Some(m_progress);

            let handle = thread::spawn(move || {
                let b = m_board.lock().unwrap().clone();

                for _ in 0..iterations_per_thread {
                    let (res, _) = b.run_many_balls(num_balls_per_run, &m_progress);
                    results.lock().unwrap().extend(res);
                }
            });

            handles.push(handle);
        }

        let mut force_stop = false;
        let mut curr_bad_secs: usize = 0;

        loop {
            thread::sleep(Duration::from_secs(1));
            let curr_it_per_s = progress.lock().unwrap().curr_its_per_sec();
            if curr_it_per_s == 0 {
                curr_bad_secs += 1;
            }

            if curr_bad_secs == 10 {
                force_stop = true;
                break;
            }

            if progress.lock().unwrap().is_finished() {
                break;
            }
        }

        if !force_stop {
            for handle in handles {
                handle.join().unwrap();
            }
        }


        let x = results.deref().lock().unwrap().clone();
        x
    }

    pub fn run_single_ball_many_iterations(&self, iterations_per_thread: u32, num_threads: u32) -> Vec<BallRunInfo> {
        let results = Arc::new(Mutex::new(Vec::new()));
        let m_board = Arc::new(Mutex::new(self.clone()));
        let progress = Arc::new(Mutex::new(ProgressPrinter::new(iterations_per_thread * num_threads)));

        let mut handles = Vec::new();

        for _ in 0..num_threads {
            let m_board = Arc::clone(&m_board);
            let results = Arc::clone(&results);
            let m_progress = Arc::clone(&progress);

            let handle = thread::spawn(move || {
                let b = m_board.lock().unwrap().clone();

                for _ in 0..iterations_per_thread {
                    let (res, _) = b.run_single_ball();
                    results.lock().unwrap().push(res);
                    m_progress.lock().unwrap().increase();
                }
            });

            handles.push(handle);
        }

        for handle in handles {
            handle.join().unwrap();
        }

        let x = results.deref().lock().unwrap().clone();
        x
    }
}