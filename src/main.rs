extern crate nalgebra as na;

use gnuplot::{Caption, Color, Figure, PlotOption};
use na::{Vector2, Vector4};
use scan_fmt::*;

type F = f32;

const G: F = -9.81;

const MASS: F = 0.075;
const K_DRAG: F = 0.01;
const K_MAGNUS: F = 0.01;

const KD: F = K_DRAG / MASS;
const KM: F = K_MAGNUS / MASS;

const DT: F = 0.01;

// TODO: add rotation deceleration
const ROTATION_SPEED: F = 1.0;
fn main() {
    let (v, x, y): (F, F, F) = scanln_fmt!("{} {} {}", F, F, F).unwrap();

    let goal = Vector2::new(x, y);
    let a = secant_converge(goal, v);
    // let a = F::to_radians(46.9177);

    println!("optimal_angle: {}", a.to_degrees());
    let res = fly_until_x(goal[0], v, a, 1.0);
    println!("TOF: {} s", res.unwrap().1);
    plot(&get_state(v, a), goal);
    plot_y_err(goal, v);
}

fn get_derivative(state: Vector4<F>) -> Vector4<F> {
    let vx = state[2];
    let vy = state[3];
    let v = (vx * vx + vy * vy).sqrt();
    let ax = (-KD * v * vx) - (KM * ROTATION_SPEED * vy);
    let ay = G - (KD * v * vy) + (KM * ROTATION_SPEED * vx);
    Vector4::new(vx, vy, ax, ay)
}

fn rk4_step(state: Vector4<F>, dt: F) -> Vector4<F> {
    let k1 = get_derivative(state);
    let k2 = get_derivative(state + (k1 * dt / 2.0));
    let k3 = get_derivative(state + (k2 * dt / 2.0));
    let k4 = get_derivative(state + (k3 * dt));

    state + dt * ((k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0)
}

fn fly_until_x(x: F, v: F, a: F, timeout: F) -> Option<(Vector4<F>, F)> {
    let n: usize = (timeout / DT) as usize;
    let mut state = get_state(v, a);
    for i in 0..n {
        state = rk4_step(state, DT);
        if state[0] > x {
            return Some((state, DT * i as F));
        }
    }

    None
}

fn y_err(goal: Vector2<F>, v: F, a: F) -> Option<F> {
    fly_until_x(goal[0], v, a, 10.0).map(|final_state| final_state.0[1] - goal[1])
}

fn get_state(v: F, a: F) -> Vector4<F> {
    let (sin, cos) = a.sin_cos();
    Vector4::new(0.0, 0.0, v * cos, v * sin)
}

// fn parabola_angle(goal: Vector2<F>, v: F) -> F {
//     F::asin(goal[1] / goal[0] - (G * goal[0]) / (v * v)) / 2.0
// }

fn secant_get_x(b: F, c: F, goal: Vector2<F>, v: F) -> F {
    let fb = y_err(goal, v, b).unwrap();
    let fc = y_err(goal, v, c).unwrap(); // TODO: proper error handling
    b - (fb) / ((fb - fc) / (b - c))
}

fn secant_converge(goal: Vector2<F>, v: F) -> F {
    let mut c; // TODO: better initial angles?
    let mut b = F::atan2(goal[1], goal[0]);
    let mut a = b + 5.0;
    for _ in 0..16 {
        c = b;
        b = a;
        a = secant_get_x(b, c, goal, v);
    }
    a
}

fn plot(initial_state: &Vector4<F>, goal: Vector2<F>) {
    let mut state = *initial_state;
    let (x, y): (Vec<f32>, Vec<f32>) = (0..256)
        .map(|_| {
            state = rk4_step(state, 0.01);
            (state[0], state[1])
        })
        .unzip();

    let mut fg = Figure::new();
    fg.axes2d()
        .set_x_axis(true, &[PlotOption::Caption("x")])
        .set_y_axis(true, &[PlotOption::Caption("y")])
        .points(
            [goal[0]],
            [goal[1]],
            &[
                PlotOption::Color(gnuplot::ColorType::RGBInteger(0xff, 0, 0)),
                PlotOption::PointSymbol('x'),
            ],
        )
        .lines(
            &x,
            &y,
            &[Caption("Projectile"), Color(gnuplot::ColorType::Black)],
        );

    fg.show().unwrap();
}

fn plot_y_err(goal: Vector2<F>, v: F) {
    let r: F = F::to_radians(90.0);
    let l: F = 0.0;
    // let l: F = F::atan2(goal[1], goal[0]);
    let step = (r - l) / 512.0;
    let a = l;
    let (x, y): (Vec<F>, Vec<F>) = (1..=512)
        .map(|i| a + i as F * step)
        .filter_map(|current_a| y_err(goal, v, current_a).map(|err| (current_a.to_degrees(), err)))
        .unzip();
    println!("amongus");

    let mut fg = Figure::new();
    fg.axes2d()
        .set_x_axis(true, &[PlotOption::Caption("Angle")])
        .set_y_axis(true, &[PlotOption::Caption("Y error")])
        .lines(
            &x,
            &y,
            &[Caption("Y error"), Color(gnuplot::ColorType::Black)],
        );
    fg.show().unwrap();
}
