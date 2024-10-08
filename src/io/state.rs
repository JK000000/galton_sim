use std::fs::File;
use std::io::Write;
use json::{JsonValue, object};
use crate::core::environment::SystemState;


pub fn state_to_json(state: &SystemState) -> JsonValue {

    let mut balls = json::JsonValue::new_array();

    for b in state.balls.iter() {
        balls.push(
            object!{
                pos_x: b.shape.pos.x,
                pos_y: b.shape.pos.y,
                r: b.shape.r,
                velocity_x: b.velocity.x,
                velocity_y: b.velocity.y,
                mass: b.mass,
            }
        ).unwrap();
    }

    object!{
        time: state.time,
        balls: balls
    }
}

pub fn history_to_json(history: &[SystemState]) -> String {
    let mut res = json::JsonValue::new_array();

    for state in history.iter() {
        res.push(state_to_json(state)).unwrap();
    }

    res.to_string()
}

pub fn history_to_json_limit_fps(history: &[SystemState], fps: f64) -> String {

    let mut curr_frame_t = 0.0;

    let mut res = json::JsonValue::new_array();

    for i in 0..(history.len()-1) {

        while curr_frame_t < history[i].time {
            curr_frame_t += 1.0/fps;
        }

        if history[i].time <= curr_frame_t && curr_frame_t <= history[i+1].time {
            if curr_frame_t - history[i].time < history[i+1].time - curr_frame_t {
                res.push(state_to_json(&history[i])).unwrap();
            } else {
                res.push(state_to_json(&history[i+1])).unwrap();
            }

            curr_frame_t += 1.0/fps;
        }
    }

    res.to_string()
}

pub fn save_history_to_file(history: &[SystemState], filename: &str, fps_limit: Option<f64>) -> std::io::Result<()> {
    let data = match fps_limit {
        None => {history_to_json(history)}
        Some(fps) => {history_to_json_limit_fps(history, fps)}
    };
    let mut file = File::create(filename)?;
    file.write(data.as_bytes())?;

    Ok(())
}