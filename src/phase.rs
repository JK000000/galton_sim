use std::fs::File;
use std::io::BufWriter;
use std::ops::Deref;
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::thread;
use rand::Rng;
use crate::board::Board;
use crate::io::config::{PhaseDiagramConfig, SimConfig};
use crate::io::output::ProgressPrinter;

#[derive(Clone)]
struct PixelResult {
    pixel_x: u32,
    pixel_y: u32,
    value: Vec<f64>,
    stddev: Vec<f64>
}

pub fn run_phase_diagram(board: Board, phase_diagram_config: PhaseDiagramConfig) {
    let results = Arc::new(Mutex::new(Vec::new()));
    let m_board = Arc::new(Mutex::new(board.clone()));

    let num_pixels = phase_diagram_config.image_width * phase_diagram_config.image_height;

    let progress = Arc::new(Mutex::new(ProgressPrinter::new(num_pixels)));

    let mut handles = Vec::new();

    let x_pixel_range = (1.0 / phase_diagram_config.image_width as f64) * phase_diagram_config.x_displacement_scale;
    let y_pixel_range = (1.0 / phase_diagram_config.image_height as f64) * phase_diagram_config.y_velocity_scale;

    for thread_idx in 0..phase_diagram_config.num_threads {
        let m_board = Arc::clone(&m_board);
        let results = Arc::clone(&results);
        let m_progress = Arc::clone(&progress);

        let handle = thread::spawn(move || {

            let mut rng = rand::thread_rng();

            let b = m_board.lock().unwrap().clone();

            for pixel_idx in (thread_idx..num_pixels).step_by(phase_diagram_config.num_threads as usize) {
                let pixel_x = pixel_idx % phase_diagram_config.image_width;
                let pixel_y = pixel_idx / phase_diagram_config.image_width;

                let x_offset = ((pixel_x as f64) / (phase_diagram_config.image_width as f64) - 0.5) * phase_diagram_config.x_displacement_scale;
                let y_offset = (((phase_diagram_config.image_height - pixel_y - 1) as f64) / (phase_diagram_config.image_height as f64)) * phase_diagram_config.y_velocity_scale;

                let mut value = vec![0.0; board.num_rows as usize];
                let mut stddev = vec![0.0; board.num_rows as usize];

                for _ in 0..phase_diagram_config.runs_per_pixel {
                    let mut start_x: f64 = rng.gen();
                    let mut start_v_y: f64 = rng.gen();

                    start_x -= 0.5;
                    start_v_y = -start_v_y;

                    start_x *= x_pixel_range;
                    start_v_y *= y_pixel_range;

                    start_x += x_offset;
                    start_v_y -= y_offset;

                    let (res, _) = b.run_single_ball_with_initial_conditions(start_x, 0.0, 0.0, start_v_y);

                    for i in 0..board.num_rows as usize {
                        value[i] += res.row_exit_pos[i] as f64;
                        stddev[i] += (res.row_exit_pos[i] as f64).powi(2);
                    }
                }

                for i in 0..board.num_rows as usize {
                    value[i] /= phase_diagram_config.runs_per_pixel as f64;
                    stddev[i] /= phase_diagram_config.runs_per_pixel as f64;
                    stddev[i] -= value[i].powi(2);
                    stddev[i] = stddev[i].sqrt();
                }

                results.lock().unwrap().push(PixelResult {
                    pixel_x,
                    pixel_y,
                    value,
                    stddev
                });
                m_progress.lock().unwrap().increase();
            }
        });

        handles.push(handle);
    }

    for handle in handles {
        handle.join().unwrap();
    }

    let mut x = results.deref().lock().unwrap().clone();

    for row_num in 0..board.num_rows as usize {

        let mut img = image::RgbImage::new(phase_diagram_config.image_width, phase_diagram_config.image_height);

        let mut min_val = 0.0;
        let mut max_val = 0.0;

        for f in x.iter() {
            if f.value[row_num] < min_val {
                min_val = f.value[row_num];
            }

            if f.value[row_num] > max_val {
                max_val = f.value[row_num];
            }
        }

        println!("Row {} Min {} Max {}", row_num, min_val, max_val);

        let grad = colorgrad::CustomGradient::new()
            .html_colors(&["deeppink", "gold", "seagreen"])
            .domain(&[min_val, max_val])
            .build().unwrap();


        for f in x.iter() {

            let mut grayness = 7.0 * f.stddev[row_num] / (max_val - min_val);

            if grayness > 1.0 {
                grayness = 1.0;
            }

            let color = grad.at(f.value[row_num]).to_rgba8();

            let mut r = color[0];
            let mut g = color[1];
            let mut b = color[2];

            r = (r as f64 * (1.0 - grayness) + 128.0 * grayness) as u8;
            g = (g as f64 * (1.0 - grayness) + 128.0 * grayness) as u8;
            b = (b as f64 * (1.0 - grayness) + 128.0 * grayness) as u8;

            img.put_pixel(f.pixel_x, f.pixel_y, image::Rgb([r,g,b]));
        }

        let mut m_filename = phase_diagram_config.output_file.clone();

        m_filename += &row_num.to_string();

        m_filename += ".png";

        img.save(&m_filename).unwrap();
    }
}

