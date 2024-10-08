use std::env;
use std::fmt::format;
use crate::calibration::run_calibration;
use crate::io::config::{read_config_from_file, SimType};
use crate::io::environment::save_environment_to_file;
use crate::io::output::save_data_files;
use crate::io::state::save_history_to_file;
use crate::phase::run_phase_diagram;

pub mod calibration;
pub mod core;
pub mod io;
pub mod board;

pub mod phase;

fn main() {

    let args: Vec<String> = env::args().collect();

    let sim_config;

    if args.len() < 2 {
        //println!("Pass configuration file path as first argument");
        for i in 1..17 {
            let v = i as f64 * 0.1;
            let h = v*v*0.5/9.81;
            let (cal_res, env, hist) = run_calibration(0.002, 0.3e2, h);
            println!("{:.4},{:.4}", v, cal_res);
            //save_environment_to_file(&env, format!("env{}.json", i).as_str()).unwrap();
            //save_history_to_file(&hist, format!("history{}.json", i).as_str(), Some(240.0)).unwrap();
        }
        return;
    } else {
        sim_config = read_config_from_file(&args[1]).unwrap();
    }

    match sim_config.sim_type {
        SimType::SingleBallVisualization(type_config) => {
            save_environment_to_file(&sim_config.board.get_environment(), "env.json").unwrap();
            let (_, history) = sim_config.board.run_single_ball();
            save_history_to_file(&history, "history.json", Some(type_config.fps)).unwrap();
        }
        SimType::SingleBallData(type_config) => {
            let res = sim_config.board.run_single_ball_many_iterations(type_config.num_runs_per_thread, type_config.num_threads);
            save_data_files(&res).unwrap();
        }
        SimType::FunnelMultiBallVisualization(type_config) => {
            let (env, _) = sim_config.board.get_environment_and_initial_state_with_funnel(&type_config.funnel_type, type_config.num_ball_rows);
            let (_, history) = sim_config.board.run_many_balls_with_funnel(type_config.num_ball_rows, &type_config.funnel_type);
            save_environment_to_file(&env, "env.json").unwrap();
            save_history_to_file(&history, "history.json", Some(type_config.fps)).unwrap();
        }
        SimType::MultiBallData(type_config) => {
            let res = sim_config.board.run_many_balls_many_iterations(type_config.num_runs_per_thread, type_config.num_threads, type_config.num_balls_per_run);
            save_data_files(&res).unwrap();
        }
        SimType::MultiBallVisualization(type_config) => {
            save_environment_to_file(&sim_config.board.get_environment(), "env.json").unwrap();
            let (_, history) = sim_config.board.run_many_balls(type_config.num_balls, &None);
            save_history_to_file(&history, "history.json", Some(type_config.fps)).unwrap();
        }
        SimType::PhaseDiagram(type_config) => {
            run_phase_diagram(sim_config.board, type_config);
        }
    }
}
