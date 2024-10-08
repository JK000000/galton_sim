use std::fs::File;
use std::io::Write;
use std::ops::Deref;
use json::object;
use crate::core::environment::Environment;
use crate::core::shapes::Shape;

impl Environment {
    pub fn to_json(&self) -> String {

        let mut shapes = json::JsonValue::new_array();

        for body in self.fixed_bodies.iter() {
            for shape in body.shapes.iter() {
                match shape.borrow().deref() {
                    Shape::Sphere(s) => {
                        shapes.push(
                            object! {
                                "type": "sphere",
                                material: 1,
                                r: s.r,
                                pos_x: s.pos.x,
                                pos_y: s.pos.y
                            }
                        ).unwrap();
                    }
                    Shape::Triangle(t) => {
                        shapes.push(
                            object! {
                                "type": "triangle",
                                material: 1,
                                v1_x: t.vertices[0].x,
                                v1_y: t.vertices[0].y,
                                v2_x: t.vertices[1].x,
                                v2_y: t.vertices[1].y,
                                v3_x: t.vertices[2].x,
                                v3_y: t.vertices[2].y
                            }
                        ).unwrap();
                    }
                    Shape::DebugBox(_) => {}
                }
            }
        }

        shapes.to_string()
    }
}

pub fn save_environment_to_file(env: &Environment, filename: &str) -> std::io::Result<()> {
    let res = env.to_json();
    let mut file = File::create(filename)?;
    file.write(res.as_bytes())?;

    Ok(())
}