extern crate nalgebra as na;

use gnuplot::{Caption, Color, Figure};
use na::Vector4;
use scan_fmt::*;

type F = f32;

const G: F = -9.81;
// const G: F = 0.0;

// const M: F = 0.075;
const KD: F = 0.2;
const KM: F = 0.1;

// TODO: add rotation deceleration
const ROTATION_SPEED: F = 1.0;
fn main() {
    let (v, a): (F, F) = scanln_fmt!("{} {}", F, F).unwrap();

    // state {x, y, vx, vy}
    let initial_state = {
        let (sin, cos) = a.to_radians().sin_cos();
        Vector4::new(0.0, 0.0, v * cos, v * sin)
    };

    // dbg!(state, rk4_step(state, dt));
    plot(&initial_state);
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
