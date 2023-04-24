#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

class TrajectoryGenerator {
public:
    TrajectoryGenerator(double x0, double x1, double v0, double v1, double a_max, double v_max, double dt) 
    {
        // Calculate the time interval and number of steps
        double d = x1 - x0;
        double t_total = max(abs(d) / v_max, sqrt(2 * abs(d) / a_max));
        int n_steps = (int)(t_total / dt) + 1;
        double t_step = t_total / n_steps;

        // Calculate the acceleration and velocity profiles
        double a = (2 * d / (t_total * t_total)) - (2 * v0 / t_total) - (2 * v1 / t_total);
        double b = (3 * v0 + a * t_total - 3 * v1) / (t_total * t_total);
        double c = (-2 * v0 - a * t_total + 2 * v1) / (t_total * t_total * t_total);
        double v_max_acc = min(v_max, v0 + a_max * t_total / 2);
        double v_max_dec = min(v_max, v1 + a_max * t_total / 2);
        double t_acc = (v_max_acc - v0) / a_max;
        double t_dec = (v_max_dec - v1) / a_max;
        double d_acc = v0 * t_acc + 0.5 * a_max * t_acc * t_acc;
        double d_dec = v1 * t_dec + 0.5 * a_max * t_dec * t_dec;
        double d_cruise = d - d_acc - d_dec;
        double t_cruise = d_cruise / v_max_acc;
        double a_profile, v_profile;
        for (int i = 0; i < n_steps; i++) {
            double t = i * t_step;
            if (t <= t_acc) {
                a_profile = a_max;
                v_profile = v0 + 0.5 * a_max * t * t;
            } else if (t <= t_acc + t_cruise) {
                a_profile = 0.0;
                v_profile = v_max_acc;
            } else {
                a_profile = -a_max;
                v_profile = v1 + v_max_dec * (t_total - t) - 0.5 * a_max * (t_total - t) * (t_total - t);
            }
            a_profiles.push_back(a_profile);
            v_profiles.push_back(v_profile);
        }
    }

    vector<double> getAccelerationProfile() {
        return a_profiles;
    }

    vector<double> getVelocityProfile() {
        return v_profiles;
    }

private:
    vector<double> a_profiles;
    vector<double> v_profiles;
};
int main() {
    // Initialize the starting position, velocity, and acceleration
    double x = 0.0;
    double v = 0.0;
    double a = 0.0;

    // Initialize the maximum acceleration, speed, and jerk
    double a_max = 5.0;
    double v_max = 1.2;
    double j_max = 3000.0;

    // Initialize the time step for updating the position and velocity
    double dt = 0.001;

    // Initialize the trajectory generator with the desired start and end points, velocities, and acceleration limits
    TrajectoryGenerator traj(x, 0.2, v, 0.0, a_max, v_max, dt);

    // Get the acceleration and velocity profiles from the trajectory generator
    vector<double> a_profile = traj.getAccelerationProfile();
    vector<double> v_profile = traj.getVelocityProfile();

    // Simulate the motion using the generated profiles
    for (int i = 0; i < a_profile.size(); i++) {
        // Calculate the new acceleration and velocity using the current acceleration and velocity, and the jerk-limited acceleration
        double j = (a_profile[i] - a) / dt;
        if (j > j_max) {
            j = j_max;
        } else if (j < -j_max) {
            j = -j_max;
        }
        double a_new = a + j * dt;
        if (a_new > a_max) {
            a_new = a_max;
        } else if (a_new < -a_max) {
            a_new = -a_max;
        }
        double v_new = v + a * dt;
        if (v_new > v_max) {
            v_new = v_max;
        } else if (v_new < -v_max) {
            v_new = -v_max;
        }
        x += v * dt + 0.5 * a * dt * dt;
        a = a_new;
        v = v_new;

        // Print the current position, velocity, and acceleration
        cout << "x = " << x << ", v = " << v << ", a = " << a << endl;
    }

    return 0;
}

