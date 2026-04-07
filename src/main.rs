#![allow(dead_code, unused)]
extern crate nalgebra as na;

// use gnuplot::{
//     Axes2D, AxesCommon, Caption, Color, ColorType, Figure,
//     PlotOption::{self, LineWidth},
// };

use gnuplot::*;
use na::{Point2, Vector4};
use scan_fmt::*;

type F = f64;

const G: F = -9.81;

const MASS: F = 0.075;
const K_DRAG: F = 0.1;
const K_MAGNUS: F = 0.01;

const KD: F = K_DRAG / MASS;
const KM: F = K_MAGNUS / MASS;

const DT: F = 0.001;

// TODO: add rotation deceleration
const ROTATION_SPEED: F = 1.0;
fn main() {
    let (v, x, y): (F, F, F) = scanln_fmt!("{} {} {}", F, F, F).unwrap();

    let goal = Point2::new(x, y);

    let mut fg = Figure::new();
    let mut ax = fg.axes2d();
    ax.set_x_range(Fix(0.0), Auto);
    ax.set_y_range(Fix(0.0), Auto);
    // fg.set_terminal("svg size 800,600", "trajectories.svg");
    plot_point(ax, Point2::new(0.0, 0.0), ColorType::RGBInteger(255, 0, 0));
    plot_and_find_trajectory(ax, v, goal, ColorType::Black);
    plot_and_find_trajectory(ax, v - 10.0, goal, ColorType::RGBInteger(255, 0, 0));
    plot_and_find_trajectory(ax, v - 20.0, goal, ColorType::RGBInteger(0, 255, 0));
    plot_point(ax, goal, ColorType::RGBInteger(255, 0, 0));
    fg.show().unwrap();
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

fn y_err(goal: Point2<F>, v: F, a: F) -> Option<F> {
    fly_until_x(goal.x, v, a, 100.0).map(|final_state| final_state.0.y - goal.y)
}

fn get_state(v: F, a: F) -> Vector4<F> {
    let (sin, cos) = a.sin_cos();
    Vector4::new(0.0, 0.0, v * cos, v * sin)
}

fn secant_get_x(b: F, c: F, goal: Point2<F>, v: F) -> F {
    let fb = y_err(goal, v, b).unwrap();
    let fc = y_err(goal, v, c).unwrap(); // TODO: proper error handling
    b - (fb) / ((fb - fc) / (b - c))
}

const N_ITER_MAX: usize = 32;
const PRECISION: F = 0.0001;

fn secant_converge(goal: Point2<F>, v: F) -> F {
    let mut c; // TODO: better initial angles?
    let mut b = F::atan2(goal.y, goal.x);
    let mut a = b + 5.0;
    for _ in 0..N_ITER_MAX {
        c = b;
        b = a;
        if (b - c).abs() < PRECISION {
            break;
        }
        a = secant_get_x(b, c, goal, v);
    }
    a
}

fn secant_converge_dbg(goal: Point2<F>, v: F, a: F, b: F) -> (F, Vec<F>) {
    let mut c; // TODO: better initial angles?
    let mut b = b;
    let mut a = a;
    let mut ye;
    let mut yes = Vec::<F>::new();
    for _ in 0..N_ITER_MAX {
        c = b;
        b = a;
        a = secant_get_x(b, c, goal, v);
        ye = y_err(goal, v, a).unwrap_or(-100.0);
        dbg!(a.to_degrees(), ye);
        yes.push(ye);
        if ye.abs() < PRECISION {
            break;
        }
    }
    (a, yes)
}

fn plot_and_find_trajectory<'a>(
    ax: &'a mut Axes2D,
    v: F,
    goal: Point2<F>,
    color: ColorType<&str>,
) -> &'a mut Axes2D {
    let a = secant_converge(goal, v);
    dbg!(y_err(goal, v, a));
    plot_trajectory(ax, v, a, color)
}

fn plot_trajectory<'a>(ax: &'a mut Axes2D, v: F, a: F, color: ColorType<&str>) -> &'a mut Axes2D {
    let mut state = get_state(v, a);
    let (x, y): (Vec<F>, Vec<F>) = (0..1024)
        .map(|i| {
            if i != 0 {
                state = rk4_step(state, DT);
            }
            (state[0], state[1])
        })
        .unzip();
    plot_lines(ax, x, y, color)
}

fn plot_lines<'a>(
    ax: &'a mut Axes2D,
    x: Vec<F>,
    y: Vec<F>,
    color: ColorType<&str>,
) -> &'a mut Axes2D {
    ax.lines(&x, &y, &[Color(color), LineWidth(8.0)]);
    ax
}

fn plot_point<'a>(ax: &'a mut Axes2D, p: Point2<F>, color: ColorType<&str>) -> &'a mut Axes2D {
    ax.points([p.x], [p.y], &[Color(color), LineWidth(8.0)]);
    ax
}

fn plot(initial_state: &Vector4<F>, goal: Point2<F>) {
    let mut state = *initial_state;
    let (x, y): (Vec<F>, Vec<F>) = (0..256)
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
            [goal.x],
            [goal.y],
            &[
                // PlotOption::Color(gnuplot::ColorType::RGBInteger(0xff, 0, 0)),
                PlotOption::PointSymbol('x'),
                LineWidth(8.0),
            ],
        )
        .lines(
            &x,
            &y,
            &[
                Caption("Projectile"),
                Color(gnuplot::ColorType::Black),
                LineWidth(8.0),
            ],
        );

    fg.show().unwrap();
}

fn plot_y_err(goal: Point2<F>, v: F) {
    let r: F = F::to_radians(90.0);
    let l: F = 0.0;
    // let l: F = F::atan2(goal[1], goal[0]);
    let step = (r - l) / 2048.0;
    let a = l;
    let (x, y): (Vec<F>, Vec<F>) = (1..=2048)
        .map(|i| a + i as F * step)
        .filter_map(|current_a| y_err(goal, v, current_a).map(|err| (current_a.to_degrees(), err)))
        .unzip();
    println!("amongus");

    let mut fg = Figure::new();
    fg.axes2d()
        .set_grid_options(false, &[])
        .set_x_axis(true, &[PlotOption::Caption("Angle")])
        .set_y_axis(true, &[PlotOption::Caption("Y error")])
        .lines(
            &x,
            &y,
            &[
                Caption("Y error"),
                Color(gnuplot::ColorType::Black),
                LineWidth(8.0),
            ],
        );
    fg.show().unwrap();
}
