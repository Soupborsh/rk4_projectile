extern crate nalgebra as na;

use gnuplot::{Caption, Color, Figure};
use na::{Vector2, Vector4};
use scan_fmt::*;

type F = f32;

const G: F = -9.81;

const MASS: F = 0.075;
const K_DRAG: F = 0.0;
const K_MAGNUS: F = 0.0;

const KD: F = K_DRAG / MASS;
const KM: F = K_MAGNUS / MASS;

const DT: F = 0.01;

// TODO: add rotation deceleration
const ROTATION_SPEED: F = 1.0;
fn main() {
    let v: F = scanln_fmt!("{}", F).unwrap();

    // state {x, y, vx, vy}

    let a = aim_angle_at(Vector2::new(4.0, 2.0), v);
    let state = {
        let (sin, cos) = a.sin_cos();
        Vector4::new(0.0, 0.0, v * cos, v * sin)
    };
    // dbg!(state, rk4_step(state, dt));
    plot(&state);
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

fn fly_until_x(initial_state: &Vector4<F>, x: F, timeout: F) -> Option<Vector4<F>> {
    let n: usize = (timeout / DT) as usize;
    let mut state = *initial_state;
    for _ in 0..n {
        state = rk4_step(state, DT);
        if state[0] > x {
            return Some(state);
        }
    }

    None
}

fn bs_check_angle(goal: Vector2<F>, v: F, a: F) -> bool {
    let initial_state = {
        let (sin, cos) = a.sin_cos();
        Vector4::new(0.0, 0.0, v * cos, v * sin)
    };
    let res = fly_until_x(&initial_state, goal[0], 1.0);
    if res.is_none() {
        false
    } else {
        (res.unwrap()[1] - goal[1]) > 0.0
    }
}

fn aim_angle_at(goal: Vector2<F>, v: F) -> F {
    let mut r: F = F::to_radians(90.0);
    let mut l: F = F::atan2(goal[1], goal[0]);
    let mut m = (l + r) / 2.0;
    for _ in 0..32 {
        if !bs_check_angle(goal, v, m) {
            r = m;
        } else {
            l = m;
        }
        m = (l + r) / 2.0;
    }
    m
    // let state = {
    //     let (sin, cos) = a.sin_cos();
    //     Vector4::new(0.0, 0.0, v * cos, v * sin)
    // };
    // let result = fly_until_x(&state, goal[0], 1.0).unwrap();

    // None
}

fn plot(initial_state: &Vector4<F>) {
    let mut state = *initial_state;
    let (x, y): (Vec<f32>, Vec<f32>) = (0..256)
        .map(|_| {
            state = rk4_step(state, 0.01);
            (state[0], state[1])
        })
        .unzip();

    let mut fg = Figure::new();
    fg.axes2d().lines(
        &x,
        &y,
        &[Caption("Projectile"), Color(gnuplot::ColorType::Black)],
    );

    fg.show().unwrap();
}
