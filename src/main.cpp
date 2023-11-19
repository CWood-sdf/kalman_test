#include "autodiff.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <vector>
using namespace std;
typedef std::shared_ptr<BackwardAutoDiff<double>> ADP;
typedef BackwardAutoDiff<double> AD;

const size_t state_size = 2;
const size_t control_size = 1;
const size_t measurement_size = 1;
const double dt = 0.001;
const double alt_stddev = 10;
const double acc_stddev = 1;

/// State vector
Eigen::Vector<double, state_size> state = Eigen::Vector<double, state_size>({
    {0, 0}
});

/// Process noise vector
Eigen::Vector<double, state_size> w = Eigen::Vector<double, state_size>({
    {0, 0}
});

/// State covariance matrix
Eigen::Matrix<double, state_size, state_size> P =
    Eigen::Matrix<double, state_size, state_size>({
        {100,   0},
        {  0, 100},
 // {  0,   0, 100}
});

/// State transition matrix
// Eigen::Matrix<double, state_size, state_size> F =
// 	Eigen::Matrix<double, state_size, state_size>({
// 		{1, dt},
// 		{0,  1},
// });

/// Control matrix
// Eigen::Matrix<double, state_size, control_size> G =
// 	Eigen::Matrix<double, state_size, control_size>({ { pow(dt, 3) / 6.0 },
//                                                       { pow(dt, 2) / 2.0 }
//                                                       });

/// Base noise covariance matrix
// Eigen::Matrix<double, state_size, state_size> Q_base =
// 	Eigen::Matrix<double, state_size, state_size>(
// 		{{measurement_stddev * measurement_stddev, 0}, {0, 0}}) *
// 	0.1;
/// Process noise covariance matrix
Eigen::Matrix<double, state_size, state_size> Q =
    Eigen::Matrix<double, state_size, state_size>();

/// Measurement matrix
// Eigen::Matrix<double, measurement_size, state_size> H =
// 	Eigen::Matrix<double, measurement_size, state_size>({
// 		{		  1,  0, 0},
//         {dt * dt / 2, dt, 1}
// });

/// Measurement noise covariance matrix
Eigen::Matrix<double, measurement_size, measurement_size> R =
    Eigen::Matrix<double, measurement_size, measurement_size>(
        { { alt_stddev * alt_stddev } }
    );

struct TrajectoryPoint {
    double actual_altitude;
    double measured_altitude;
    double motor_force;
    double kalman_altitude = 0;
    double kalman_stddev = 0;
    double measured_acceleration = 0;
    double t;
    TrajectoryPoint(
        double actual_altitude, double measured_altitude, double motor_force,
        double t, double measured_acceleration
    )
      : actual_altitude(actual_altitude), measured_altitude(measured_altitude),
        motor_force(motor_force), measured_acceleration(measured_acceleration),
        t(t) {
    }
};

Eigen::Vector<ADP, state_size> state_transition(
    Eigen::Vector<ADP, state_size> state,
    Eigen::Vector<ADP, control_size> control
) {
    Eigen::Vector<ADP, state_size> ret = Eigen::Vector<ADP, state_size>();
    // control(0) is t
    ret(0) = state(0) + state(1) * AD::makeConst(dt);

    auto deltaV = (cos(control(0)) + AD::makeConst(-0.01)) * AD::makeConst(133);
    ret(1) = state(1) + deltaV * AD::makeConst(dt);
    return ret;
}
Eigen::Vector<ADP, measurement_size>
state_expection(Eigen::Vector<ADP, state_size> state) {
    Eigen::Vector<ADP, measurement_size> ret =
        Eigen::Vector<ADP, measurement_size>();
    ret(0) = state(0);
    return ret;
}

std::vector<TrajectoryPoint> generate_rocket_trajectory(
    double dt_sim, double motor_force, double motor_lifetime,
    double gravity_down, double drag, double motor_stddev,
    double motor_cuttoff_rate
) {
    std::vector<TrajectoryPoint> ret;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> normals =
        std::normal_distribution<double>(0, alt_stddev);
    std::normal_distribution<double> acc_normals =
        std::normal_distribution<double>(0, acc_stddev);
    std::normal_distribution<double> motor_noise =
        std::normal_distribution<double>(0, motor_stddev);
    double t = 0;
    double alt = 0.1;
    double vel = 0;
    bool force_checked = false;
    auto get_motor_force = [&t, &motor_force, &motor_lifetime, &gen,
                            &motor_noise, motor_cuttoff_rate,
                            &force_checked]() {
        if (t < motor_lifetime) {
            return motor_force + motor_noise(gen);
        } else if (t < motor_lifetime + motor_force / motor_cuttoff_rate) {
            double new_force =
                motor_force * (1 - (motor_cuttoff_rate * (t - motor_lifetime)));
            if (new_force < 0) {
                new_force = 0;
            }
            if (new_force > motor_force) {
                new_force = 0;
                if (!force_checked) {
                    cout << "Motor force is too high, "
                            " motor will never cut off "
                         << endl;
                    force_checked = true;
                }
            }
            return new_force + motor_noise(gen) * new_force / motor_force;
        } else {
            return 0.0;
        }
    };
    double lastTCheck = 0;
    double tCheckDist = 1.0;
    double recordInterval = dt;
    double lastRecord = 0;
    while (alt >= 0) {
        // double motor_force = get_motor_force();
        // double acc = motor_force - gravity_down;
        // double drag_acc = std::abs(drag * vel * vel);
        // if (vel < 0) {
        // 	acc += drag_acc;
        // } else {
        // 	acc -= drag_acc;
        // }
        // double deltaV = acc * dt_sim;
        double deltaV = (cos(t) - 0.01) * 133;
        if (t >= lastRecord + recordInterval) {
            ret.push_back(TrajectoryPoint(
                alt, alt + normals(gen), motor_force, t,
                deltaV * dt_sim + acc_normals(gen)
            ));
            lastRecord = t;
        }

        vel += deltaV * dt_sim;
        alt += vel * dt_sim;

        t += dt_sim;
        if (t >= lastTCheck + tCheckDist) {
            cout << "t: " << t << ", alt: " << alt << ", vel: "
                 << vel
                 // << ", acc: " << acc << ", drag_acc: " << drag_acc
                 << ", dv: " << deltaV << endl;
            lastTCheck = t;
        }
        if (t > 200) {
            ret = {};
            break;
        }
    }
    return ret;
}

int main() {
    cout << std::fixed;
    cout << std::setprecision(2);
    auto shutoffT = 10;
    auto shutoffRate = 0.1;
    auto motorForce = 70;
    auto trajectory = generate_rocket_trajectory(
        0.00001, motorForce, shutoffT, 29.8, 0.1, 0, shutoffRate
    );

    int i = 0;
    // since we're assuming the jerk is constant, we base the covariances off of
    // it

    // Position is the third integral of jerk
    // double jerk_to_pos = std::pow(dt, 3) / 6;
    // // Velocity is the second integral of jerk
    // double jerk_to_vel = std::pow(dt, 2) / 2;
    // // Acceleration is the first integral of jerk
    // double jerk_to_acc = dt;
    // double v_x = jerk_to_pos * jerk_to_pos;
    // double v_v = jerk_to_vel * jerk_to_vel;
    // double v_a = jerk_to_acc * jerk_to_acc;
    // double v_xv = jerk_to_pos * jerk_to_vel;
    // double v_xa = jerk_to_pos * jerk_to_acc;
    // double v_va = jerk_to_vel * jerk_to_acc;
    // // the number multiplying Q is the only random number in the whole
    // program it is the standard deviation of jerk
    Q = Eigen::Matrix<double, state_size, state_size>({
            {1, 1},
            {1, 1},
    }) *
        pow(2.7, 2);
    auto control = Eigen::Vector<ADP, control_size>({ { AD::makeConst(0) } });
    auto state_ad = Eigen::Vector<ADP, state_size>();
    for (size_t i = 0; i < state_size; i++) {
        state_ad(i) = AD::makeConst(state(i));
    }

    Eigen::Vector<ADP, state_size> predicted_state_ad =
        state_transition(state_ad, control);
    Eigen::Vector<double, state_size> predicted_state =
        Eigen::Vector<double, state_size>();
    Eigen::Matrix<double, state_size, state_size> F =
        Eigen::Matrix<double, state_size, state_size>();
    // calculate expected state
    for (size_t i = 0; i < state_size; i++) {
        predicted_state_ad(i)->reset();
        predicted_state_ad(i)->forward();
        predicted_state_ad(i)->gradient = 1;
        predicted_state(i) = predicted_state_ad(i)->output.value();
        AD::backward(predicted_state_ad(i));
        for (size_t j = 0; j < state_size; j++) {
            F(j, i) = state_ad(j)->get_gradient();
        }
    }

    Eigen::Matrix<double, state_size, state_size> predicted_P =
        F * P * F.transpose() + Q;
    double meanOfSquErr = 0;
    double meanOfMeasErr = 0;
    for (auto& point : trajectory) {
        // get our control input
        Eigen::Vector<ADP, control_size> control;
        control =
            Eigen::Vector<ADP, control_size>({ { AD::makeConst(point.t) } });
        // } else if (point.t < shutoffT + motorForce / shutoffRate) {
        // 	control = Eigen::Vector<double, control_size>({ { -shutoffRate *
        // 	                                                  motorForce }
        // });
        // }

        auto alt = point.measured_altitude;
        // auto acc = point.measured_acceleration;
        auto measurement = Eigen::Vector<double, measurement_size>({ { alt } });

        Eigen::Vector<ADP, state_size> predicted_state_ad =
            Eigen::Vector<ADP, state_size>();
        for (size_t i = 0; i < state_size; i++) {
            predicted_state_ad(i) = AD::makeConst(predicted_state(i));
        }
        Eigen::Vector<ADP, measurement_size> expected_state_ad =
            state_expection(predicted_state_ad);

        Eigen::Vector<double, measurement_size> expected_state =
            Eigen::Vector<double, measurement_size>();

        Eigen::Matrix<double, measurement_size, state_size> H =
            Eigen::Matrix<double, measurement_size, state_size>();
        // calculate expected state
        for (size_t i = 0; i < measurement_size; i++) {
            expected_state_ad(i)->reset();
            expected_state_ad(i)->forward();
            expected_state_ad(i)->gradient = 1;
            expected_state(i) = expected_state_ad(i)->output.value();
            AD::backward(expected_state_ad(i));
            for (size_t j = 0; j < state_size; j++) {
                H(i, j) = predicted_state_ad(j)->get_gradient();
            }
        }

        Eigen::Vector<double, measurement_size> residual =
            measurement - expected_state;

        auto residual_covariance = H * predicted_P * H.transpose() + R;
        // make kalman gain
        auto kalman_gain =
            predicted_P * H.transpose() * residual_covariance.inverse();
        // update the state using kalman gain
        state = predicted_state + kalman_gain * residual;

        auto err = state(0) - point.actual_altitude;
        meanOfSquErr += err * err;
        auto measErr = point.measured_altitude - point.actual_altitude;
        meanOfMeasErr += measErr * measErr;
        // update the state covariance matrix
        auto predict_p_help =
            (Eigen::Matrix<double, state_size, state_size>::Identity() -
             kalman_gain * H);
        // P = predict_p_help * predicted_P * predict_p_help.transpose() +
        //     kalman_gain * R * kalman_gain.transpose();
        P = predict_p_help * predicted_P;

        // predict next state
        predicted_state_ad = state_transition(state_ad, control);
        predicted_state = Eigen::Vector<double, state_size>();
        F = Eigen::Matrix<double, state_size, state_size>();
        // calculate expected state
        for (size_t i = 0; i < state_size; i++) {
            predicted_state_ad(i)->reset();
            predicted_state_ad(i)->forward();
            predicted_state_ad(i)->gradient = 1;
            predicted_state(i) = predicted_state_ad(i)->output.value();
            AD::backward(predicted_state_ad(i));
            for (size_t j = 0; j < state_size; j++) {
                F(j, i) = state_ad(j)->gradient.value();
            }
        }

        predicted_P = F * P * F.transpose() + Q;
        // predicted_state = F * state + G * control + w;
        // cout << predicted_state(0) << endl;
        // predicted_P = F * P * F.transpose() + Q;
        trajectory[i].kalman_altitude = state(0);
        trajectory[i].kalman_stddev = sqrt(P(0, 0));
        i++;
    }
    cout << "Mean of Squared Error: " << meanOfSquErr / trajectory.size()
         << endl;
    cout << "Mean of Measurement Error: " << meanOfMeasErr / trajectory.size()
         << endl;
    cout << "Kalman improvement: " << 100 - meanOfSquErr / meanOfMeasErr * 100
         << "%" << endl;

    auto file = ofstream();
    file.open("./graph/js/data.js");
    double print_interval = 0.1;
    double last_print = 0;

    if (file.is_open()) {
        file << std::fixed;
        file << std::setprecision(2);
        file << "var data = [\n";
        for (auto& point : trajectory) {
            if (point.t < last_print + print_interval) {
                continue;
            }
            last_print = point.t;
            file << "  [" << point.actual_altitude << ", "
                 << point.measured_altitude << ", " << point.motor_force << ", "
                 << point.kalman_altitude << ", " << point.kalman_stddev
                 << "],\n";
        }
        file << "];" << endl;
    } else {
        cout << "Unable to open file";
    }
    file.close();

    auto x = AD::makeConst(3.14 / 2 * 3);
    ADP y = sin(x);

    y->forward();
    y->gradient = 1;
    AD::backward(y);
    cout << y->output.value() << endl;
    cout << x->gradient.value() << endl;

    //
    // auto x2 = AD::makeConst(2);
    // auto y2 = AD::makeConst(3);
    // auto x1 = AD::makeConst(3);
    // auto y1 = AD::makeConst(4);
    //
    // Eigen::Matrix<ADP, 2, 2> A = Eigen::Matrix<ADP, 2, 2>({
    // 	{x1, y1},
    //        {x2, y2}
    //    });
    //
    // auto z1 = AD::makeConst(3);
    // auto z2 = AD::makeConst(2);
    // Eigen::Vector<ADP, 2> b = Eigen::Vector<ADP, 2>({
    // 	{z1, z2}
    //    });
    //
    // Eigen::Vector<ADP, 2> res = A * b;
    //
    // auto res0 = res(0);
    // res0->forward();
    // auto res1 = res(1);
    // res1->forward();
    //
    // res(0)->gradient = 1;
    // res(1)->gradient = 1;
    // AD::backward(res(0));
    // AD::backward(res(1));
    // cout << res(0)->output.value() << endl;
    // cout << x2->gradient.value() << endl;
}
