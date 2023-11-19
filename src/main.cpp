#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <vector>
using namespace std;

const size_t state_size = 3;
const size_t control_size = 1;
const size_t measurement_size = 2;
const double dt = 0.001;
const double alt_stddev = 10;
const double acc_stddev = 1;

Eigen::Vector<double, state_size> state = Eigen::Vector<double, state_size>({
	{0, 0, 0}
});

/// Process noise vector
Eigen::Vector<double, state_size> w = Eigen::Vector<double, state_size>({
	{0, 0, 0}
});

/// State covariance matrix
Eigen::Matrix<double, state_size, state_size> P =
	Eigen::Matrix<double, state_size, state_size>({
		{500, 500,   0},
        {500, 500,   0},
        {500,   0, 500}
});

/// State transition matrix
Eigen::Matrix<double, state_size, state_size> F =
	Eigen::Matrix<double, state_size, state_size>({
		{1, dt, dt* dt / 2.0},
        {0,  1,           dt},
        {0,  0,            1}
});

/// Control matrix
Eigen::Matrix<double, state_size, control_size> G =
	Eigen::Matrix<double, state_size, control_size>(
		{{pow(dt, 3) / 6.0}, {pow(dt, 2) / 2.0}, {dt}}
	);

/// Base noise covariance matrix
// Eigen::Matrix<double, state_size, state_size> Q_base =
// 	Eigen::Matrix<double, state_size, state_size>(
// 		{{measurement_stddev * measurement_stddev, 0}, {0, 0}}) *
// 	0.1;
/// Process noise covariance matrix
Eigen::Matrix<double, state_size, state_size> Q =
	Eigen::Matrix<double, state_size, state_size>();

/// Measurement matrix
Eigen::Matrix<double, measurement_size, state_size> H =
	Eigen::Matrix<double, measurement_size, state_size>({
		{		  1,  0, 0},
        {dt * dt / 2, dt, 1}
});

/// Measurement noise covariance matrix
Eigen::Matrix<double, measurement_size, measurement_size> R =
	Eigen::Matrix<double, measurement_size, measurement_size>({
		{alt_stddev * alt_stddev,                      0},
        {                      0, acc_stddev* acc_stddev}
});

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
	double tCheckDist = 0.5;
	double recordInterval = dt;
	double lastRecord = 0;
	while (alt >= 0) {
		double motor_force = get_motor_force();
		double acc = motor_force - gravity_down;
		double drag_acc = std::abs(drag * vel * vel);
		if (vel < 0) {
			acc += drag_acc;
		} else {
			acc -= drag_acc;
		}
		if (t >= lastRecord + recordInterval) {
			ret.push_back(TrajectoryPoint(
				alt, alt + normals(gen), motor_force, t, acc + acc_normals(gen)
			));
			lastRecord = t;
		}
		vel += acc * dt_sim;
		alt += vel * dt_sim;

		t += dt_sim;
		if (t >= lastTCheck + tCheckDist) {
			cout << "t: " << t << ", alt: " << alt << ", vel: " << vel
				 << ", acc: " << acc << ", drag_acc: " << drag_acc << endl;
			lastTCheck = t + tCheckDist;
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
	auto shutoffT = 2;
	auto shutoffRate = 0.1;
	auto motorForce = 60;
	auto trajectory = generate_rocket_trajectory(
		0.00001, motorForce, shutoffT, 1.8, 0.1, 0, shutoffRate
	);

	// vector<double> altitudes = {-32.4, -11.1, 18, 22.9,
	// 	19.5, 28.5, 46.5, 68.9, 48.2, 56.1, 90.5, 104.9,
	// 	140.9, 148, 187.6, 209.2, 244.6, 276.4, 323.5,
	// 	357.3, 357.4, 398.3, 446.7, 465.1, 529.4, 570.4,
	// 	636.8, 693.3, 707.3, 748.5};
	//
	// vector<double> accelerations = {39.72, 40.02, 39.97,
	// 	39.81, 39.75, 39.6, 39.77, 39.83, 39.73, 39.87,
	// 	39.81, 39.92, 39.78, 39.98, 39.76, 39.86, 39.61,
	// 	39.86, 39.74, 39.87, 39.63, 39.67, 39.96, 39.8,
	// 	39.89, 39.85, 39.9, 39.81, 39.81, 39.68};
	// trajectory = {};
	// for (int i = 0; i < altitudes.size(); i++) {
	// 	trajectory.push_back(TrajectoryPoint(altitudes[i],
	// 		altitudes[i], 0, 0, accelerations[i]));
	// }

	int i = 0;
	// Q = F * Q_base * F.transpose();
	//
	double jerk_to_pos = std::pow(dt, 3) / 6;
	double jerk_to_vel = std::pow(dt, 2) / 2;
	double jerk_to_acc = dt;
	double v_x = jerk_to_pos * jerk_to_pos;
	double v_v = jerk_to_vel * jerk_to_vel;
	double v_a = jerk_to_acc * jerk_to_acc;
	double v_xv = jerk_to_pos * jerk_to_vel;
	double v_xa = jerk_to_pos * jerk_to_acc;
	double v_va = jerk_to_vel * jerk_to_acc;
	Q = Eigen::Matrix<double, state_size, state_size>({
			{ v_x, v_xv, v_xa},
            {v_xv,  v_v, v_va},
            {v_xa, v_va,  v_a}
    }) *
	    pow(3.7, 2);
	auto control = Eigen::Vector<double, control_size>({{0}});
	Eigen::Vector<double, state_size> predicted_state =
		F * state + G * control + w;
	Eigen::Matrix<double, state_size, state_size> predicted_P =
		F * P * F.transpose() + Q;
	for (auto& point : trajectory) {
		Eigen::Vector<double, control_size> control;
		if (point.t < shutoffT) {
			control = Eigen::Vector<double, control_size>({{0}});
		} else if (point.t < shutoffT + motorForce / shutoffRate) {
			control = Eigen::Vector<double, control_size>(
				{{-shutoffRate * motorForce}}
			);
		} else {
			control = Eigen::Vector<double, control_size>({{0}});
		}

		auto alt = point.measured_altitude;
		auto acc = point.measured_acceleration;
		auto measurement = Eigen::Vector<double, measurement_size>({
			{alt, acc}
        });

		auto kalman_gain = predicted_P * H.transpose() *
		                   (H * predicted_P * H.transpose() + R).inverse();

		state =
			predicted_state + kalman_gain * (measurement - H * predicted_state);

		auto predict_p_help =
			(Eigen::Matrix<double, state_size, state_size>::Identity() -
		     kalman_gain * H);
		P = predict_p_help * predicted_P * predict_p_help.transpose() +
		    kalman_gain * R * kalman_gain.transpose();

		predicted_state = F * state + G * control + w;
		// cout << predicted_state(0) << endl;
		predicted_P = F * P * F.transpose() + Q;
		trajectory[i].kalman_altitude = state(0);
		trajectory[i].kalman_stddev = sqrt(P(0, 0));
		i++;
	}

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
}
