use std::fs::File;
use std::io;
use std::io::Read;
use glam::DVec2;
use crate::board::{Board, BoardMaterialsData, FunnelType, PegSpacing};
use crate::board::PegType::{Circle, Triangle};
use crate::core::environment::ExternalAccelerationMode;

pub struct SimConfig {
    pub board: Board,
    pub sim_type: SimType,
}

pub enum SimType {
    SingleBallVisualization(SingleBallVisualizationConfig),
    SingleBallData(SingleBallDataConfig),
    FunnelMultiBallVisualization(FunnelMultiBallVisualizationConfig),
    MultiBallData(MultiBallDataConfig),
    MultiBallVisualization(MultiBallVisualizationConfig),
    PhaseDiagram(PhaseDiagramConfig)
}

pub struct PhaseDiagramConfig {
    pub image_width: u32,
    pub image_height: u32,
    pub runs_per_pixel: u32,
    pub x_displacement_scale: f64,
    pub y_velocity_scale: f64,
    pub num_threads: u32,
    pub output_file: String
}

pub struct SingleBallVisualizationConfig {
    pub fps: f64,
}

pub struct SingleBallDataConfig {
    pub num_threads: u32,
    pub num_runs_per_thread: u32,

}

pub struct MultiBallVisualizationConfig {
    pub fps: f64,
    pub num_balls: u32,
}

pub struct FunnelMultiBallVisualizationConfig {
    pub fps: f64,
    pub num_ball_rows: u32,
    pub funnel_type: FunnelType,
}

pub struct MultiBallDataConfig {
    pub num_balls_per_run: u32,
    pub num_threads: u32,
    pub num_runs_per_thread: u32,
}

const PARSE_ERROR: &str = "Error occurred when parsing configuration file.";

fn parse_config(data: &str) -> SimConfig {
    let parsed_json = json::parse(data).expect(PARSE_ERROR);

    let sim_type_json = parsed_json["sim_type"].as_str().expect(PARSE_ERROR);

    let sim_type: SimType;

    if sim_type_json == "single_visualization" {
        sim_type = SimType::SingleBallVisualization(SingleBallVisualizationConfig {
            fps: parsed_json["sim_type_config"]["fps"].as_f64().expect(PARSE_ERROR)
        });
    } else if sim_type_json == "single_data" {
        sim_type = SimType::SingleBallData(SingleBallDataConfig {
            num_threads: parsed_json["sim_type_config"]["num_threads"].as_u32().expect(PARSE_ERROR),
            num_runs_per_thread: parsed_json["sim_type_config"]["num_runs_per_thread"].as_u32().expect(PARSE_ERROR),
        });
    } else if sim_type_json == "funnel_multi_visualization" {
        let funnel_type: FunnelType;

        if parsed_json["sim_type_config"]["funnel_type"].as_str().expect(PARSE_ERROR) == "simple" {
            funnel_type = FunnelType::Simple(parsed_json["sim_type_config"]["funnel_angle"].as_f64().expect(PARSE_ERROR));
        } else {
            panic!("{}", PARSE_ERROR);
        }

        sim_type = SimType::FunnelMultiBallVisualization(FunnelMultiBallVisualizationConfig {
            fps: parsed_json["sim_type_config"]["fps"].as_f64().expect(PARSE_ERROR),
            num_ball_rows: parsed_json["sim_type_config"]["num_ball_rows"].as_u32().expect(PARSE_ERROR),
            funnel_type,
        });
    } else if sim_type_json == "multi_data" {
        sim_type = SimType::MultiBallData(MultiBallDataConfig {
            num_balls_per_run: parsed_json["sim_type_config"]["num_balls_per_run"].as_u32().expect(PARSE_ERROR),
            num_threads: parsed_json["sim_type_config"]["num_threads"].as_u32().expect(PARSE_ERROR),
            num_runs_per_thread: parsed_json["sim_type_config"]["num_runs_per_thread"].as_u32().expect(PARSE_ERROR),
        });
    } else if sim_type_json == "multi_visualization" {
        sim_type = SimType::MultiBallVisualization(MultiBallVisualizationConfig {
            fps: parsed_json["sim_type_config"]["fps"].as_f64().expect(PARSE_ERROR),
            num_balls: parsed_json["sim_type_config"]["num_balls_per_run"].as_u32().expect(PARSE_ERROR)
        });
    } else if sim_type_json == "phase_diagram" {
        sim_type = SimType::PhaseDiagram(PhaseDiagramConfig {
            image_width: parsed_json["sim_type_config"]["image_width"].as_u32().expect(PARSE_ERROR),
            image_height: parsed_json["sim_type_config"]["image_height"].as_u32().expect(PARSE_ERROR),
            runs_per_pixel: parsed_json["sim_type_config"]["runs_per_pixel"].as_u32().expect(PARSE_ERROR),
            x_displacement_scale: parsed_json["sim_type_config"]["x_displacement_scale"].as_f64().expect(PARSE_ERROR),
            y_velocity_scale: parsed_json["sim_type_config"]["y_velocity_scale"].as_f64().expect(PARSE_ERROR),
            num_threads: parsed_json["sim_type_config"]["num_threads"].as_u32().expect(PARSE_ERROR),
            output_file: String::from(parsed_json["sim_type_config"]["output_file"].as_str().expect(PARSE_ERROR))
        });
    } else {
        panic!("{}", PARSE_ERROR);
    }

    let peg_type;

    if parsed_json["board"]["peg_type"].as_str().expect(PARSE_ERROR) == "circle" {
        peg_type = Circle;
    } else if parsed_json["board"]["peg_type"].as_str().expect(PARSE_ERROR) == "triangle" {
        peg_type = Triangle;
    } else {
        panic!("{}", PARSE_ERROR);
    }

    let peg_spacing;

    let peg_spacing_json = parsed_json["board"]["peg_spacing"].as_str().expect(PARSE_ERROR);

    if peg_spacing_json == "regular" {
        peg_spacing = PegSpacing::Regular(parsed_json["board"]["peg_spacing_param"].as_f64().expect(PARSE_ERROR));
    } else if peg_spacing_json == "wider_on_top" {
        peg_spacing = PegSpacing::WiderOnTop(parsed_json["board"]["peg_spacing_param"].as_f64().expect(PARSE_ERROR));
    } else {
        panic!("{}", PARSE_ERROR);
    }

    let shaking_mode;

    let shaking_mode_json = parsed_json["board"]["shaking_mode"].as_str().expect(PARSE_ERROR);

    if shaking_mode_json == "disabled" {
        shaking_mode = ExternalAccelerationMode::Disabled;
    }
    else if shaking_mode_json == "sin" {

        let x = parsed_json["board"]["shaking_mode_amp_x"].as_f64().expect(PARSE_ERROR);
        let y = parsed_json["board"]["shaking_mode_amp_y"].as_f64().expect(PARSE_ERROR);
        let f = parsed_json["board"]["shaking_mode_freq"].as_f64().expect(PARSE_ERROR);

        shaking_mode = ExternalAccelerationMode::Sinusoidal(DVec2::new(x,y), f);
    }
    else {
        panic!();
    }

    SimConfig {
        board: Board {
            peg_type,
            peg_spacing,
            row_spacing: parsed_json["board"]["row_spacing"].as_f64().expect(PARSE_ERROR),
            peg_size: parsed_json["board"]["peg_size"].as_f64().expect(PARSE_ERROR),
            ball_size: parsed_json["board"]["ball_size"].as_f64().expect(PARSE_ERROR),
            ball_density: parsed_json["board"]["ball_density"].as_f64().expect(PARSE_ERROR),
            num_rows: parsed_json["board"]["num_rows"].as_u32().expect(PARSE_ERROR),
            materials_data: BoardMaterialsData {
                ball_peg_stiffness: parsed_json["board"]["ball_peg_stiffness"].as_f64().expect(PARSE_ERROR),
                ball_peg_damping: parsed_json["board"]["ball_peg_damping"].as_f64().expect(PARSE_ERROR),
                ball_ball_stiffness: parsed_json["board"]["ball_ball_stiffness"].as_f64().expect(PARSE_ERROR),
                ball_ball_damping: parsed_json["board"]["ball_ball_damping"].as_f64().expect(PARSE_ERROR),
            },
            shaking_mode
        },
        sim_type
    }
}

pub fn read_config_from_file(filename: &str) -> std::io::Result<SimConfig> {
    let mut file = File::open(filename)?;
    let mut data = String::new();
    file.read_to_string(&mut data)?;
    let config = parse_config(&data);

    Ok(config)
}

pub fn read_config_from_stdin() -> SimConfig {
    let mut data = String::new();
    let mut stdin = io::stdin();
    stdin.read_to_string(&mut data).unwrap();
    parse_config(&data)
}
