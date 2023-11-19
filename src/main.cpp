#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <random>
#include <stack>
#include <unordered_set>
#include <vector>
using namespace std;

template <class T> class ReverseTopologicalSort {
public:
	static std::vector<std::shared_ptr<T>>
	reverseTopoSort(const std::shared_ptr<T>& startNode) {
		std::vector<std::shared_ptr<T>> result;
		std::unordered_set<std::shared_ptr<T>> visited;
		std::stack<std::shared_ptr<T>> stack;

		reverseTopoSortUtil(startNode, visited, stack);

		while (!stack.empty()) {
			result.push_back(stack.top());
			stack.pop();
		}

		return result;
	}

private:
	static void reverseTopoSortUtil(
		const std::shared_ptr<T>& node,
		std::unordered_set<std::shared_ptr<T>>& visited,
		std::stack<std::shared_ptr<T>>& stack
	) {
		visited.insert(node);

		for (const auto& parent : node->inputs) {
			if (visited.find(parent) == visited.end()) {
				reverseTopoSortUtil(parent, visited, stack);
			}
		}

		stack.push(node);
	}
};
template <typename T> struct BackwardAutoDiff {
	typedef BackwardAutoDiff<T> Self;
	enum OpTp {
		Const,
		Add,
		// Sub,
		Mul,
		// Div,
	};
	T constInput;
	vector<std::shared_ptr<BackwardAutoDiff<T>>> inputs;
	std::optional<T> output;
	const OpTp op;
	std::optional<T> gradient;
	BackwardAutoDiff(OpTp op) : op(op) {
	}
	BackwardAutoDiff(T val) : op(Const), constInput(val) {
	}
	static std::shared_ptr<Self> makeConst(T val) {
		auto ret = std::make_shared<Self>(Const);
		ret->constInput = val;
		return ret;
	}
	friend std::shared_ptr<Self>
	operator+(std::shared_ptr<Self> a, std::shared_ptr<BackwardAutoDiff> b) {
		auto ret = std::make_shared<Self>(Add);
		ret->inputs.push_back(a);
		ret->inputs.push_back(b);
		return ret;
	}
	friend std::shared_ptr<Self>
	operator*(std::shared_ptr<Self> a, std::shared_ptr<BackwardAutoDiff> b) {
		auto ret = std::make_shared<Self>(Mul);
		ret->inputs.push_back(a);
		ret->inputs.push_back(b);
		return ret;
	}

	void forward() {
		if (this->output.has_value()) {
			return;
		}
		switch (this->op) {
		case Const:
			this->output = this->constInput;
			break;
		case Add:
			this->inputs[0]->forward();
			this->inputs[1]->forward();
			this->output = this->inputs[0]->output.value() +
			               this->inputs[1]->output.value();
			break;
		case Mul:
			this->inputs[0]->forward();
			this->inputs[1]->forward();
			this->output = this->inputs[0]->output.value() *
			               this->inputs[1]->output.value();
			break;
		}
	}
	static void backward(std::shared_ptr<Self> node) {
		auto reversed = ReverseTopologicalSort<Self>::reverseTopoSort(node);
		for (std::shared_ptr<Self>& node : reversed) {
			for (std::shared_ptr<Self>& input : node->inputs) {
				if (!input->gradient.has_value()) {
					input->gradient = 0;
				}
				input->gradient.value();
			}
			if (!node->gradient.has_value()) {
				node->gradient = 0;
			}
			switch (node->op) {
			case Const:
				break;
			case Add:
				node->inputs[0]->gradient.value() += node->gradient.value();
				node->inputs[1]->gradient.value() += node->gradient.value();
				break;
			case Mul:
				node->inputs[0]->gradient.value() +=
					node->gradient.value() * node->inputs[1]->output.value();
				node->inputs[1]->gradient.value() +=
					node->gradient.value() * node->inputs[0]->output.value();
				break;
			}
		}
	}
};

// int main() {
// 	auto x = BackwardAutoDiff<double>::makeConst(2);
// 	auto y = BackwardAutoDiff<double>::makeConst(3);
// 	auto z = x * x + x * y;
// 	z->forward();
// 	z->gradient = 1;
// 	BackwardAutoDiff<double>::backward(z);
// 	cout << z->output.value() << endl;
// 	cout << x->gradient.value() << endl;
// 	cout << y->gradient.value() << endl;
// }
const size_t state_size = 3;
const size_t control_size = 1;
const size_t measurement_size = 2;
const double dt = 0.001;
const double alt_stddev = 10;
const double acc_stddev = 1;

/// State vector
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
		{100,   0,   0},
        {  0, 100,   0},
        {  0,   0, 100}
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
		{ { pow(dt, 3) / 6.0 }, { pow(dt, 2) / 2.0 }, { dt } }
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
	auto shutoffT = 10;
	auto shutoffRate = 0.1;
	auto motorForce = 70;
	auto trajectory = generate_rocket_trajectory(
		0.00001, motorForce, shutoffT, 29.8, 0.1, 0, shutoffRate
	);

	int i = 0;
	// since we're assuming the jerk is constant, we base the covariances off of
	// that

	// Position is the third integral of jerk
	double jerk_to_pos = std::pow(dt, 3) / 6;
	// Velocity is the second integral of jerk
	double jerk_to_vel = std::pow(dt, 2) / 2;
	// Acceleration is the first integral of jerk
	double jerk_to_acc = dt;
	double v_x = jerk_to_pos * jerk_to_pos;
	double v_v = jerk_to_vel * jerk_to_vel;
	double v_a = jerk_to_acc * jerk_to_acc;
	double v_xv = jerk_to_pos * jerk_to_vel;
	double v_xa = jerk_to_pos * jerk_to_acc;
	double v_va = jerk_to_vel * jerk_to_acc;
	// the number multiplying Q is the only random number in the whole program
	// it is the standard deviation of jerk
	Q = Eigen::Matrix<double, state_size, state_size>({
			{ v_x, v_xv, v_xa},
            {v_xv,  v_v, v_va},
            {v_xa, v_va,  v_a}
    }) *
	    pow(2.7, 2);
	auto control = Eigen::Vector<double, control_size>({ { 0 } });
	Eigen::Vector<double, state_size> predicted_state =
		F * state + G * control + w;
	Eigen::Matrix<double, state_size, state_size> predicted_P =
		F * P * F.transpose() + Q;
	double meanOfSquErr = 0;
	double meanOfMeasErr = 0;
	for (auto& point : trajectory) {
		// get our control input
		Eigen::Vector<double, control_size> control;
		if (point.t < shutoffT) {
			control = Eigen::Vector<double, control_size>({ { 0 } });
		} else if (point.t < shutoffT + motorForce / shutoffRate) {
			control = Eigen::Vector<double, control_size>({ { -shutoffRate *
			                                                  motorForce } });
		} else {
			control = Eigen::Vector<double, control_size>({ { 0 } });
		}

		auto alt = point.measured_altitude;
		auto acc = point.measured_acceleration;
		auto measurement = Eigen::Vector<double, measurement_size>({
			{alt, acc}
        });

		// make kalman gain
		auto kalman_gain = predicted_P * H.transpose() *
		                   (H * predicted_P * H.transpose() + R).inverse();
		// update the state using kalman gain
		state =
			predicted_state + kalman_gain * (measurement - H * predicted_state);

		auto err = state(0) - point.actual_altitude;
		meanOfSquErr += err * err;
		auto measErr = point.measured_altitude - point.actual_altitude;
		meanOfMeasErr += measErr * measErr;
		// update the state covariance matrix
		auto predict_p_help =
			(Eigen::Matrix<double, state_size, state_size>::Identity() -
		     kalman_gain * H);
		P = predict_p_help * predicted_P * predict_p_help.transpose() +
		    kalman_gain * R * kalman_gain.transpose();

		// predict next state
		predicted_state = F * state + G * control + w;
		// cout << predicted_state(0) << endl;
		predicted_P = F * P * F.transpose() + Q;
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

	// typedef std::shared_ptr<BackwardAutoDiff<double>> ADP;
	// typedef BackwardAutoDiff<double> AD;
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
