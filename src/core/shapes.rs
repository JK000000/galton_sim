use std::f64::consts::PI;
use glam::{DVec2, DVec3};
use crate::core::tree::BoundingBox;


#[derive(Clone)]
pub struct SphereData {
    pub(crate) r: f64,
    pub(crate) pos: DVec2
}

#[derive(Clone)]
pub struct TriangleData {
    pub(crate) vertices: [DVec2; 3]
}

pub enum Shape {
    Sphere(SphereData),
    Triangle(TriangleData),
    DebugBox(BoundingBox)
}

impl Shape {
    pub fn get_bounding_box(&self) -> BoundingBox {
        match self {
            Shape::Sphere(s) => {
                BoundingBox {
                    point_low: DVec2 { x: s.pos.x - s.r, y: s.pos.y - s.r},
                    point_high: DVec2 { x: s.pos.x + s.r, y: s.pos.y + s.r}
                }
            }
            Shape::Triangle(t) => {
                BoundingBox {
                    point_low: DVec2 { x: t.vertices[0].x.min(t.vertices[1].x).min(t.vertices[2].x), y: t.vertices[0].y.min(t.vertices[1].y).min(t.vertices[2].y) },
                    point_high: DVec2 { x: t.vertices[0].x.max(t.vertices[1].x).max(t.vertices[2].x), y: t.vertices[0].y.max(t.vertices[1].y).max(t.vertices[2].y)}
                }
            }
            Shape::DebugBox(b) => { b.clone() }
        }
    }
    pub fn get_intersection(&self, other: &Shape) -> Option<IntersectionInfo> {
        match self {
            Shape::Sphere(s) => {
                match other {
                    Shape::Sphere(s2) => {
                        intersection_sphere_sphere(s, s2)
                    }
                    Shape::Triangle(t) => {
                        let res = intersection_triangle_sphere(t, s);
                        if let Some(mut r) = res {
                            r.normal = -r.normal;
                            Some(r)
                        } else {
                            res
                        }
                    }
                    Shape::DebugBox(_) => {
                        unimplemented!()
                    }
                }
            }
            Shape::Triangle(t) => {
                match other {
                    Shape::Sphere(s) => {
                        intersection_triangle_sphere(t, s)
                    }
                    Shape::Triangle(_) => {
                        unimplemented!()
                    }
                    Shape::DebugBox(_) => {
                        unimplemented!()
                    }
                }
            }
            Shape::DebugBox(_) => {
                unimplemented!()
            }
        }
    }
}

fn circular_segment_area(r_l: f64, r_s: f64) -> f64 {
    PI* r_l * r_l /2.0 - r_l * r_l *f64::asin(r_s / r_l) - r_s *f64::sqrt(r_l * r_l - r_s * r_s)
}

fn circular_segment_area_in_terms_of_chord(r: f64, c: f64) -> f64 {
    PI* r * r /2.0 - r * r *f64::asin(f64::sqrt(1.0 - c*c/(4.0* r * r))) - (c/2.0) * f64::sqrt(r * r - c*c/4.0)
}

fn intersection_sphere_sphere(a: &SphereData, b: &SphereData) -> Option<IntersectionInfo> {
    let d = (a.pos - b.pos).length();
    let r1 = a.r;
    let r2 = b.r;

    if r1 + r2 < d {
        return None;
    }

    let d1 = (r1 * r1 - r2 * r2 + d*d)/(2.0*d);
    let d2 = d - d1;

    let mut area = circular_segment_area(r1, d1) + circular_segment_area(r2, d2);

    let normal = (b.pos - a.pos).normalize();

    if area < 0.0 {
        area = 0.0;
    }

    Some(IntersectionInfo {
        area,
        normal,
        center: a.pos + normal * d1
    })
}

fn triangle_area(p1: &DVec2, p2: &DVec2, p3: &DVec2) -> f64 {
    let a = *p1 - *p3;
    let b = *p2 - *p3;

    let a = DVec3 { x: a.x, y: b.y, z: 0.0};
    let b = DVec3 { x: b.x, y: b.y, z: 0.0};

    a.cross(b).length() / 2.0
}

fn segment_circle_intersection(p1: &DVec2, p2: &DVec2, s: &SphereData) -> Vec<DVec2> {

    let center_projection = (s.pos - *p1).project_onto(*p2 - *p1) + *p1;

    let h = (s.pos - center_projection).length();

    let d = f64::sqrt(s.r*s.r - h*h);

    let res1 = center_projection + (*p1 - *p2).normalize() * d;
    let res2 = center_projection - (*p1 - *p2).normalize() * d;

    let l = (*p1 - *p2).length();

    let mut res = Vec::new();

    if (res1 - *p1).length() + (res1 - *p2).length() - l < f64::EPSILON {
        res.push(res1);
    }

    if (res2 - *p1).length() + (res2 - *p2).length() - l < f64::EPSILON {
        res.push(res2);
    }

    res
}

fn intersection_triangle_sphere(t: &TriangleData, s: &SphereData) -> Option<IntersectionInfo> {
    let mut intersection_points : Vec<(u8, DVec2)> = Vec::new();

    intersection_points.extend(segment_circle_intersection(&t.vertices[0], &t.vertices[1], s).iter().map(|x| (0, *x)).collect::<Vec<(u8, DVec2)>>());
    intersection_points.extend(segment_circle_intersection(&t.vertices[1], &t.vertices[2], s).iter().map(|x| (1, *x)).collect::<Vec<(u8, DVec2)>>());
    intersection_points.extend(segment_circle_intersection(&t.vertices[0], &t.vertices[2], s).iter().map(|x| (2, *x)).collect::<Vec<(u8, DVec2)>>());

    // Remove duplicates
    let mut indices_to_remove = Vec::new();

    for i in 1..intersection_points.len() {
        for j in 0..i {
            if (intersection_points[i].1 - intersection_points[j].1).length() < 1e-10 {
                indices_to_remove.push(j);
                break;
            }
        }
    }

    intersection_points = intersection_points.iter().enumerate().filter(|(i, _)| !indices_to_remove.contains(i)).map(|(_, x)| *x).collect();

    if intersection_points.len() > 2 {
        panic!();
    }

    if intersection_points.len() < 2 {
        return None;
    }

    let (sa, pa) = intersection_points[0];
    let (sb, pb) = intersection_points[1];

    if sa == sb {

        let i;
        let j;

        match sa {
            0 => {
                i = 0;
                j = 1;
            },
            1 => {
                i = 1;
                j = 2;
            },
            2 => {
                i = 0;
                j = 2;
            },
            _ => panic!()
        }

        let projected_center = (s.pos - t.vertices[j]).project_onto(t.vertices[i] - t.vertices[j]) + t.vertices[j];

        let dist_v = projected_center - s.pos;

        let dist = dist_v.length();

        let mut area = circular_segment_area(s.r, dist);
        let normal = -dist_v.normalize();

        if area < 0.0 {
            area = 0.0;
        }

        return Some(IntersectionInfo {
            area,
            normal,
            center: projected_center
        })
    }

    let center = (pa + pb) / 2.0;
    let normal = (s.pos - center).normalize();

    let vertex_idx;

    if sa == 0 && sb == 1 {
        vertex_idx = 1;
    } else if sa == 1 && sb == 2 {
        vertex_idx = 2;
    } else {
        vertex_idx = 0;
    }

    let mut area = triangle_area(&pa, &pb, &t.vertices[vertex_idx]) + circular_segment_area_in_terms_of_chord(s.r, (pa - pb).length());


    if area < 0.0 {
        area = 0.0;
    }

    Some(IntersectionInfo {
        area,
        normal,
        center
    })
}

// Result of Shape::get_intersection, contains necessary information to evaluate physical interaction between intersecting objects
#[derive(Copy, Clone)]
pub struct IntersectionInfo {
    pub(crate) area: f64,
    pub(crate) normal: DVec2,
    pub(crate) center: DVec2
}

impl IntersectionInfo {

    pub fn sum_into(&mut self, other: &IntersectionInfo) {
        let new_normal = (self.normal * self.area + other.normal * other.area).normalize();
        let new_center = (self.center * self.area + other.center * other.area) / (self.area + other.area);
        self.normal = new_normal;
        self.center = new_center;
        self.area += other.area;
    }

    pub fn sum(intersections: &[IntersectionInfo]) -> IntersectionInfo {
        assert!(intersections.len() >= 1);

        let mut res_normal = DVec2::new(0.0, 0.0);
        let mut res_center = DVec2::new(0.0, 0.0);
        let mut res_area = 0.0;

        for i in intersections {
            res_normal += i.normal * i.area;
            res_center += i.center * i.area;
            res_area += i.area;
        }

        res_normal = res_normal.normalize();
        res_center /= res_area;

        IntersectionInfo {
            area: res_area,
            normal: res_normal,
            center: res_center
        }
    }
}

#[cfg(test)]
mod tests {
    use glam::DVec2;
    use crate::core::shapes::{ Shape, SphereData, TriangleData};

    #[test]
    fn sphere_sphere_1() {
        let s1 = Shape::Sphere(SphereData { r: 3.0 , pos: DVec2 {x: 0.0, y: 0.0}});
        let s2 = Shape::Sphere(SphereData { r: 3.0, pos: DVec2 {x: 0.0, y: 4.0}});

        let res = s1.get_intersection(&s2).unwrap();

        assert_eq!(res.center, DVec2 {x: 0.0, y: 2.0});
        assert_eq!(res.normal, DVec2 {x: 0.0, y: 1.0});
        assert!((res.area - 6.1949641602235858184).abs() < 1e-7);
    }

    #[test]
    fn sphere_sphere_2() {
        let s1 = Shape::Sphere(SphereData { r: 2.5 , pos: DVec2 {x: 0.0, y: 1.0}});
        let s2 = Shape::Sphere(SphereData { r: 6.0, pos: DVec2 {x: 6.0, y: 7.0}});

        let res = s1.get_intersection(&s2).unwrap();

        assert!((res.center - DVec2 {x: 1.7604166666666663, y: 2.7604166666666661}).length() < 1e-7);
        assert!((res.normal - DVec2 {x: std::f64::consts::FRAC_1_SQRT_2, y: std::f64::consts::FRAC_1_SQRT_2 }).length() < 1e-7);
        assert!((res.area - 0.004470804180538579).abs() < 1e-7);
    }

    #[test]
    fn sphere_sphere_3() {
        let s1 = Shape::Sphere(SphereData { r: 2.5 , pos: DVec2 {x: 0.0, y: 1.0}});
        let s2 = Shape::Sphere(SphereData { r: 6.0, pos: DVec2 {x: 10.0, y: 7.0}});

        let res = s1.get_intersection(&s2);

        match res {
            None => {}
            Some(_) => { panic!() }
        }
    }

    #[test]
    fn triangle_sphere_1() {
        let t = Shape::Triangle(TriangleData { vertices: [DVec2 { x: 2.0, y: 3.0}, DVec2 { x: 5.0, y: 7.0}, DVec2 { x: 0.0, y: 10.0}] });
        let s = Shape::Sphere(SphereData {r: 0.5, pos: DVec2{x: -5.0, y: -5.0}});

        let res = t.get_intersection(&s);

        match res {
            None => {}
            Some(_) => { panic!() }
        }
    }

    #[test]
    fn triangle_sphere_2() {
        let t = Shape::Triangle(TriangleData { vertices: [DVec2 { x: -2.0, y: 0.0}, DVec2 { x: 3.0, y: 0.0}, DVec2 { x: 0.0, y: -2.0}] });
        let s = Shape::Sphere(SphereData {r: 2.001, pos: DVec2{x: 0.0, y: 2.0}});

        let res = t.get_intersection(&s);

        match res {
            None => {panic!();}
            Some(res) => {
                assert!((res.area - 0.00008434216023071558).abs() < 1e-7);
                assert!((res.center - DVec2{x: 0.0, y: 0.0}).length() < 1e-7);
                assert!((res.normal - DVec2{x: 0.0, y: 1.0}).length() < 1e-7);
            }
        }
    }

    #[test]
    fn triangle_sphere_3() {
        let t = Shape::Triangle(TriangleData { vertices: [DVec2 { x: 0.0, y: 0.0}, DVec2 { x: -1.0, y: -2.0}, DVec2 { x: 1.0, y: -2.0}] });
        let s = Shape::Sphere(SphereData {r: 2.001, pos: DVec2{x: 0.0, y: 2.0}});

        let res = t.get_intersection(&s);

        match res {
            None => {panic!();}
            Some(res) => {
                assert!((res.area - 5e-7).abs() < 1e-7);
                assert!((res.center - DVec2{x: 0.0, y: -1e-3}).length() < 1e-7);
                assert!((res.normal - DVec2{x: 0.0, y: 1.0}).length() < 1e-7);
            }
        }
    }

    #[test]
    fn triangle_sphere_4() {
        let dl = 0.06325345840347387677;

        let t = Shape::Triangle(TriangleData { vertices: [DVec2 { x: -2.0, y: 0.0}, DVec2 { x: 4.0, y: 0.0}, DVec2 { x: 0.0, y: -2.0}] });
        let s = Shape::Sphere(SphereData {r: 2.001, pos: DVec2{x: -2.0 + dl, y: 2.0}});

        let res = t.get_intersection(&s);

        match res {
            None => {panic!();}
            Some(res) => {
                assert!((res.area - 0.00008434216023071558).abs() < 1e-7);
                assert!((res.center - DVec2{x: -1.9367465415965261, y: 0.0}).length() < 1e-7);
                assert!((res.normal - DVec2{x: 0.0, y: 1.0}).length() < 1e-7);
            }
        }
    }
}