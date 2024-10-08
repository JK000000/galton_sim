use std::cell::RefCell;
use std::rc::Rc;
use glam::DVec2;
use crate::core::environment::{Ball, CollisionData, Environment, ExternalAccelerationMode, FixedBody, MaterialsData, SystemState};
use crate::core::shapes::{Shape, SphereData, TriangleData};
use crate::core::simulation::SimulationRunner;

fn get_calibration_env(ball_size: f64, damping: f64, initial_h: f64) -> (Environment, SystemState) {
    let fixed_bodies = vec![
        FixedBody {
            shapes: vec![
                Rc::new(RefCell::new(Shape::Triangle(TriangleData {
                    vertices: [
                        DVec2::new(-0.1, 0.0),
                        DVec2::new(0.1, 0.0),
                        DVec2::new(0.0, -0.1)
                    ]
                })))
            ]
        }
    ];

    let r = ball_size / 2.0;

    let initial = SystemState {
        balls: vec![
            Ball {
                id: 10,
                shape: SphereData {
                    r,
                    pos: DVec2::new(0.0, initial_h + r),
                },
                mass: 0.000111,
                velocity: DVec2::new(0.0, 0.0),
            }
        ],
        time: 0.0,
    };


    (Environment::new(fixed_bodies, MaterialsData {
        ball_ball: CollisionData {
            stiffness: 1e6,
            damping: 1e6,
        },
        ball_peg: CollisionData {
            stiffness: 1e5,
            damping,
        },
    }, ExternalAccelerationMode::Disabled),
     initial)
}

pub fn run_calibration(ball_size: f64, damping: f64, initial_h: f64) -> (f64, Environment, Vec<SystemState>) {

    let (r_env, _) = get_calibration_env(ball_size, damping, initial_h);

    let (env, initial) = get_calibration_env(ball_size, damping, initial_h);

    let mut sim = SimulationRunner::new(env);
    sim.set_state(initial);

    sim.run(2.0);

    let res = sim.get_history();

    let mut after_bounce = false;

    let mut highest_after_bounce = 0.0;

    for s in res.iter() {
        if s.balls[0].velocity.y > 0.0 {
            after_bounce = true;
        }

        if after_bounce {
            highest_after_bounce = f64::max(highest_after_bounce, s.balls[0].shape.pos.y);
        }
    }

    //println!("Highest after bounce {}", highest_after_bounce);

    let v1 = f64::sqrt(2.0 * 9.81 * initial_h);
    let v2 = f64::sqrt(2.0 * 9.81 * (highest_after_bounce - ball_size / 2.0));

    (v2/v1, r_env, Vec::from(res))
}
