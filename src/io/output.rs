use std::collections::VecDeque;
use std::fs::File;
use std::io::{stdout, Write};
use std::time::{Duration, SystemTime};
use crate::board::BallRunInfo;

pub fn save_data_files(data: &[BallRunInfo]) -> std::io::Result<()> {

    let mut file1 = File::create("ball_run_info.csv")?;

    file1.write("id,row_num,pos_idx\n".as_bytes())?;

    for (id, b) in data.iter().enumerate() {
        for row_num in 0..b.row_exit_pos.len() {
            file1.write(format!("{},{},{}\n", id,row_num,b.row_exit_pos[row_num]).as_bytes())?;
        }
    }

    Ok(())
}

pub struct ProgressPrinter {
    total: u32,
    completed: u32,
    curr_second: VecDeque<SystemTime>,
    last_print: SystemTime,
    start_time: SystemTime
}

impl ProgressPrinter {
    pub fn new(total: u32) -> ProgressPrinter {
        ProgressPrinter {
            total,
            completed: 0,
            curr_second: VecDeque::new(),
            last_print: SystemTime::now(),
            start_time: SystemTime::now()
        }
    }

    fn update(&mut self) {
        let now = SystemTime::now();
        while let Some(time) = self.curr_second.front() {
            if now.duration_since(time.clone()).unwrap() > Duration::from_secs(1) {
                self.curr_second.pop_front();
            } else {
                break;
            }
        }
    }

    fn insert(&mut self) {
        let now = SystemTime::now();

        self.update();

        self.curr_second.push_back(now);
    }

    fn print(&self) {
        let progress = (self.completed as f64) / (self.total as f64);

        print!("{}/{}", self.completed, self.total);

        let progress_i = (progress * 20.0).floor() as i32;

        print!(" [");

        for i in 0..20 {
            if i < progress_i {
                print!("=");
            }
            else {
                print!(".");
            }
        }

        print!("] ");

        print!("{}            \r", self.curr_second.len());
        stdout().flush().unwrap();
    }

    fn print_finish(&self) {
        let total_time = SystemTime::now().duration_since(self.start_time).unwrap();
        let total_speed = (self.total as f64) / (total_time.as_millis() as f64 / 1000.0);

        println!("{:.2} {}                     ", total_speed, total_time.as_secs());
    }

    pub fn increase(&mut self) {
        self.insert();
        self.completed += 1;

        if self.completed == self.total {
            self.print_finish();
        }
        else if SystemTime::now().duration_since(self.last_print.clone()).unwrap() > Duration::from_millis(100) {
            self.last_print = SystemTime::now();
            self.print();
        }
    }

    pub fn curr_its_per_sec(&mut self) -> usize {

        self.update();

        if SystemTime::now().duration_since(self.last_print.clone()).unwrap() > Duration::from_millis(100) {
            self.last_print = SystemTime::now();
            self.print();
        }

        self.curr_second.len()
    }

    pub fn is_finished(&self) -> bool {
        self.completed >= self.total
    }
}