#![allow(dead_code)]
extern crate rand;
extern crate threadpool;
use threadpool::ThreadPoolBuilder;
use std::ops::{Add, Mul, Sub};
use std::sync::{Mutex, Arc};
use rand::{Rand, Open01};
use std::io::prelude::*;
use std::sync::mpsc::{channel};

pub trait Vector: Add + Mul + Sub + Copy + Clone {
    type Vector;
    fn zero() -> Self::Vector;
}

#[derive(Debug, Clone, Copy)]
struct Vec3f{
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
impl Add<Vec3f> for Vec3f {
    type Output = Vec3f;
    fn add(self, rhs: Self) -> Self { Vec3f { x: self.x + rhs.x, y: self.y + rhs.y, z: self.z + rhs.z } }
}
impl Sub<Vec3f> for Vec3f {
    type Output = Vec3f;
    fn sub(self, rhs: Self) -> Self { Vec3f { x: self.x - rhs.x, y: self.y - rhs.y, z: self.z - rhs.z } }
}
impl Mul<f64> for Vec3f {
    type Output = Vec3f;
    fn mul(self, rhs: f64) -> Vec3f { Vec3f { x: self.x * rhs, y: self.y * rhs, z: self.z * rhs } }
}
impl Mul<Vec3f> for Vec3f {
    type Output = f64;
    fn mul(self, rhs: Vec3f) -> f64 { self.x * rhs.x + self.y * rhs.y + self.z * rhs.z }
}
impl Vector for Vec3f {
    type Vector = Self;
    fn zero() -> Vec3f { Vec3f { x: 0.0, y: 0.0, z: 0.0 } }    
}
impl Vec3f {
    pub fn new(x: f64, y: f64, z: f64) -> Vec3f { Vec3f { x: x, y: y, z: z } } 
    pub fn cross(&self, rhs: &Vec3f) -> Vec3f { Vec3f { x: self.y * rhs.z - self.z * rhs.y, y: self.z * rhs.x - self.x * rhs.z, z: self.x * rhs.y - self.y * rhs.z} }
    pub fn norm(&mut self) -> Vec3f { *self = *self * (1.0 / (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()); *self }
    pub fn mult(&self, rhs: &Vec3f) -> Vec3f { Vec3f::new(self.x * rhs.x, self.y * rhs.y, self.z * rhs.z) }
    pub fn dot(&self, rhs: Vec3f) -> f64 { self.mul(rhs) }
}

#[derive(Debug, Clone, Copy)]
struct Ray {
    pub o: Vec3f,
    pub d: Vec3f,    
}
impl Ray {
    pub fn new(o: Vec3f, d: Vec3f) -> Ray { Ray { o: o, d: d } }
}

#[derive(PartialEq, Debug, Clone, Copy)]
enum ReflT {
    DIFF,
    SPEC,
    REFR,
}
#[derive(Debug, Clone, Copy)]
struct Sphere {
    pub rad: f64,
    pub pos: Vec3f,
    pub emit: Vec3f,
    pub color: Vec3f,
    pub refl: ReflT, 
}
impl Sphere {
    pub fn new(rad: f64, p: Vec3f, e: Vec3f, c: Vec3f, refl: ReflT) -> Sphere { Sphere { rad: rad, pos: p, emit: e, color: c, refl: refl } }
    
    pub fn intersect(&self, r: &Ray) -> f64 {
        let (op, eps) = (self.pos - r.o, 1.0e-4);
        let b = op.dot(r.d);
        let mut det = b * b - op.dot(op) + self.rad * self.rad;
        if det < 0.0 { return 0.0; } else { det = det.sqrt(); }
        return if b - det > eps { b - det } else { if b + det > eps { b + det } else { 0.0 }  }
    }
}

macro_rules! clamp { ($val: expr, $low: expr, $high: expr) => ( if $val < $low { $low } else if $val > $high { $high } else { $val } ); }
#[inline(always)] 
fn to_i32(x: f64) -> i32 { 
    (clamp!(x, 0.0, 1.0).powf(1.0 / 2.2) * 255.0 + 0.5) as i32 
}
#[inline] 
fn intersect(spheres: Arc<[Sphere; 9]>, r: &Ray, t: &mut f64, id: &mut usize) -> bool {
    let inf = 1.0e20;
    *t = inf;
    for i in 0..spheres.len() {
        let d = spheres[i].intersect(r);
        if (d != 0.0) && d < *t { *t = d; *id = i; } 
    }
    *t < inf
}



fn radiance(spheres: Arc<[Sphere; 9]>, r_: &Ray, depth_: i32) -> Vec3f {
    use ReflT::*;
    let mut rng = rand::thread_rng();
    let mut t = unsafe { std::mem::uninitialized::<f64>() };
    let mut id = 0;
    let mut r = *r_;
    let mut depth = depth_;
    let mut cl = Vec3f::zero();
    let mut cf = Vec3f::new(1.0, 1.0, 1.0);
    loop {
        if !intersect(spheres.clone(), &r, &mut t, &mut id) { return cl; }
        let obj = &spheres[id];
        let x = r.o + r.d * t;
        let n = (x - obj.pos).norm();
        let nl = if n.dot(r.d) < 0.0 { n } else { n * -1.0 };
        let mut f = obj.color;
        let p = if f.x > f.y && f.x > f.z { f.x } else { if f.y > f.z { f.y } else { f.z } };
        cl = cl + cf.mult(&obj.emit);
        depth += 1;
        if depth > 5 {
            let Open01(c01)= Open01::<f64>::rand(&mut rng);   
            if c01 < p { f = f * (1.0 / p); } else { return cl; }
        }
        cf = cf.mult(&f);
        if obj.refl == DIFF {
            let Open01(tmp) = Open01::<f64>::rand(&mut rng);
            let (r1, Open01(r2)) = (2.0 * std::f64::consts::PI * tmp, Open01::<f64>::rand(&mut rng));
            let r2s = r2.sqrt();
            let w = nl; 
            let u = if w.x.abs() > 0.1 { Vec3f::new(0.0, 1.0, 0.0) } else { Vec3f::new(1.0, 0.0, 0.0).cross(&w) }.norm(); 
            let v = w.cross(&u);
            let d = (u * r1.cos() * r2s + v * r1.sin() * r2s + w * (1.0 - r2).sqrt()).norm();
            r = Ray::new(x, d);
            continue
        } else if obj.refl == SPEC {
            r = Ray::new(x, r.d - n * 2.0 * n.dot(r.d));
            continue
        } 
        let reflray = Ray::new(x, r.d - n * 2.0 * n.dot(r.d));
        let into = n.dot(nl) > 0.0;
        let (nc, nt) = (1.0, 1.5);
        let (nnt, ddn) = (if into { nc / nt } else { nt / nc }, r.d.dot(nl));
        let cos2t = 1.0 - nnt * nnt * (1.0 - ddn * ddn);
        if cos2t < 0.0 {  
            r = reflray;
            continue
        }
        let tdir = (r.d * nnt - n * ((if into { 1.0 } else { -1.0 }) * (ddn * nnt + cos2t.sqrt()))).norm();
        let (a, b, c) = (nt -nc, nt + nc, 1.0 - (if into { -ddn } else {tdir.dot(n)}));
        let r0 = a * a / (b * b);
        let re = r0 + (1.0 - r0) * c * c * c * c * c;
        let tr = 1.0 - re;
        let p = 0.25 + 0.5 * re;
        let rp = re / p;
        let tp = tr / (1.0 - p);
        let Open01(r01) = Open01::<f64>::rand(&mut rng);
        if r01 < p {
            cf = cf * rp;
            r = reflray;
        } else {
            cf = cf * tp;
            r = Ray::new(x ,tdir);
        }
        continue
    }
}

fn main() {
    use ReflT::*;
    let (w, h) = (1024, 768);
    let args: Vec<_> = std::env::args().collect();
    let samps = if args.len() == 2 || args.len() == 3 { args[1].trim().parse::<usize>().unwrap() } else { 2000 };
    let thread_num = if args.len() == 3 { args[2].trim().parse::<usize>().unwrap() } else { 5 };
    let spheres = Arc::new([
        //Scene: radius, position, emission, color, material
        Sphere::new(1.0e5, Vec3f::new( 1.0e5+1.0,40.8,81.6), Vec3f::zero(),Vec3f::new(0.75,0.25,0.25),DIFF),//Left
        Sphere::new(1.0e5, Vec3f::new(-1.0e5+99.0,40.8,81.6),Vec3f::zero(),Vec3f::new(0.25,0.25,0.75),DIFF),//Rght
        Sphere::new(1.0e5, Vec3f::new(50.0,40.8, 1.0e5),     Vec3f::zero(),Vec3f::new(0.75,0.75,0.75),DIFF),//Back
        Sphere::new(1.0e5, Vec3f::new(50.0,40.8,-1.0e5+170.0), Vec3f::zero(),Vec3f::zero(),           DIFF),//Frnt
        Sphere::new(1.0e5, Vec3f::new(50.0, 1.0e5, 81.6),    Vec3f::zero(),Vec3f::new(0.75,0.75,0.75),DIFF),//Botm
        Sphere::new(1.0e5, Vec3f::new(50.0,-1.0e5+81.6,81.6),Vec3f::zero(),Vec3f::new(0.75,0.75,0.75),DIFF),//Top
        Sphere::new(16.5,Vec3f::new(27.0,16.5,47.0),       Vec3f::zero(),Vec3f::new(1.0,1.0,1.0)*0.999, SPEC),//Mirr
        Sphere::new(16.5,Vec3f::new(73.0,16.5,78.0),       Vec3f::zero(),Vec3f::new(1.0,1.0,1.0)*0.999, REFR),//Glas
        Sphere::new(600.0, Vec3f::new(50.0,681.6-0.27,81.6),Vec3f::new(12.0,12.0,12.0),  Vec3f::zero(), DIFF) //Lite    
    ]);
    let cam = Ray::new(Vec3f::new(50.0, 52.0, 295.6), Vec3f::new(0.0, -0.042612, -1.0).norm());
    let cx = Vec3f::new(w as f64 * 0.5135 / h as f64, 0.0, 0.0);
    let cy = cx.cross(&cam.d).norm() * 0.5135;
    let mut r = Vec3f::zero();
    let vcolor = Arc::new(Mutex::new(vec![Vec3f::zero();w * h]));

    let n_workers = thread_num;
    let n_jobs = h;
    let pool = ThreadPoolBuilder::new().name("Smallpt Worker".to_string()).stack_size(4 * 1024 * 1024).num_threads(n_workers).build();

    let (tx, rx) = channel();
    let stderr = Arc::new(Mutex::new(std::io::stderr()));
    
    for y in 0..n_jobs {
        let tx = tx.clone();
        let vcolor_cp = vcolor.clone();
        let spheres = spheres.clone();
        let stderr = stderr.clone();   
        pool.execute(move || {
            let mut rng = rand::thread_rng();
            write!(stderr.lock().unwrap(),"\rRendering ({} spp) {:.*}%", samps*4, 5, 100.0 * (y) as f64/(h as f64-1.0)).unwrap();
            for x in 0..w {
                let i = (h - y - 1) * w + x;
                for sy in 0..2 {
                    for sx in 0..2 {
                        r = Vec3f::zero();
                        for _ in 0..samps {
                            let (Open01(rand1), Open01(rand2)) = (Open01::<f64>::rand(&mut rng), Open01::<f64>::rand(&mut rng));
                            let (r1, r2) = (2.0 * rand1, 2.0 * rand2); 
                            let (dx, dy) = (if r1 < 1.0 { r1.sqrt() - 1.0 } else { 1.0 - (2.0 - r1).sqrt() }, if r2 < 1.0 { r2.sqrt() - 1.0 } else { 1.0 - (2.0 - r2).sqrt() });
                            let mut d  = cx * (((sx as f64 + 0.5 + dx as f64) / 2.0 + x as f64) / w as f64 - 0.5) +
                                    cy * (((sy as f64 + 0.5 + dy as f64) / 2.0 + y as f64) / h as f64 - 0.5) + cam.d;
                            r = r + radiance(spheres.clone(), &Ray::new(cam.o + d * 140.0, d.norm()), 0) * (1.0 / samps as f64);
                        }
                        let mut vcolor_data = vcolor_cp.lock().unwrap();
                        vcolor_data[i] = vcolor_data[i] + Vec3f::new(clamp!(r.x, 0.0, 1.0), clamp!(r.y, 0.0, 1.0), clamp!(r.z, 0.0, 1.0)) * 0.25;
                    }
                }
            }
            tx.send(1).is_ok();
        });
    }

    assert_eq!(rx.iter().take(n_jobs).fold(0, |a, b| a + b), n_jobs);
    
    let mut file = std::fs::File::create(std::path::Path::new("image.ppm")).unwrap();
    let header = format!("P3\n{} {}\n255\n", w, h);
    file.write(header.as_bytes()).unwrap();
    let mut cache = String::new();
    let color = vcolor.lock().unwrap();
    for i in 0..color.len() {
        cache.push_str(&format!("{} {} {} ", to_i32(color[i].x), to_i32(color[i].y), to_i32(color[i].z)));
    }

    write!(file,"{}", cache).unwrap();
    println!("Finished");
}
