/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <algorithm>
#include <iostream>
#include <iterator>

using std::string;
using std::vector;
using std::normal_distribution;
static std::default_random_engine gen;


void ParticleFilter::init(double x, double y, double theta, double std[]) {

    //The number of particles
    num_particles = 50;

    // Creates normal  distribution
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    for (int i = 0; i < num_particles; i++) {

        Particle p;
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1.0;

        particles.push_back(p);
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {

    // Creating normal distributions
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);

    // Calculate new state.
    for (int n = 0; n < num_particles; n++) {

        double theta = particles[n].theta;

        if (fabs(yaw_rate) != 0) {

            particles[n].x += velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
            particles[n].y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
            particles[n].theta += yaw_rate * delta_t;
        } else {

            particles[n].x += velocity * delta_t * cos(theta);
            particles[n].y += velocity * delta_t * sin(theta);
        }

        // Adding noise.
        particles[n].x += dist_x(gen);
        particles[n].y += dist_y(gen);
        particles[n].theta += dist_theta(gen);
    }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs> &observations) {


    for ( int i = 0; i < observations.size(); i++) { // For each observation
        double closest_m = 1000;
        int land_id = -1;
        for (int j = 0; j < predicted.size(); j++) {
            double dx = observations[i].x - predicted[j].x;
            double dy = observations[i].y - predicted[j].y;
            double dist = pow(dx, 2) + pow(dy, 2);
            if (dist < closest_m) {
                closest_m = dist;
                land_id = predicted[j].id;
            }
        }

        observations[i].id = land_id;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {

    const double std_x = std_landmark[0];
    const double std_y = std_landmark[1];
    for (int i = 0; i < num_particles; i++) {

        double pX = particles[i].x;
        double pY = particles[i].y;
        double pTheta = particles[i].theta;


        vector<LandmarkObs> predicted;
        for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {

            int id = map_landmarks.landmark_list[j].id_i;
            float landX = map_landmarks.landmark_list[j].x_f;
            float landY = map_landmarks.landmark_list[j].y_f;


            if (pow(pX - landX, 2) + pow(pY - landY, 2) < pow(sensor_range, 2)) {
                predicted.push_back(LandmarkObs{id, landX, landY});
            }
        }

        vector<LandmarkObs> transformedObs;


        for (int j = 0; j < observations.size(); j++) {
            double xx = cos(pTheta) * observations[j].x - sin(pTheta) * observations[j].y + pX;
            double yy = sin(pTheta) * observations[j].x + cos(pTheta) * observations[j].y + pY;

            transformedObs.push_back(LandmarkObs{observations[j].id, xx, yy});
        }

        dataAssociation(predicted, transformedObs);

        particles[i].weight = 1.0;
        for (int j = 0; j < transformedObs.size(); j++) {


            double landmarkX = 0, landmarkY = 0;

            int k = 0;
            int found = 0;

            while (!found && k < predicted.size()) {
                if (predicted[k].id == transformedObs[j].id) {
                    found = 1;
                    landmarkX = predicted[k].x;
                    landmarkY = predicted[k].y;
                }
                k++;
            }

            // Calculating weight.
            double dX = transformedObs[j].x - landmarkX;
            double dY = transformedObs[j].y - landmarkY;

            double weight = (1 / (2 * M_PI * std_x * std_y)) *
                            exp(-(dX * dX / (2 * std_x * std_x) +
                                  (dY * dY / (2 * std_y * std_y))));
            if (weight == 0) {
                particles[i].weight *= 0.00001;
            } else {
                particles[i].weight *= weight;
            }
        }
    }
}

void ParticleFilter::resample() {

    // Get weights and max weight.
    vector<double> weights;
    double maxWeight = std::numeric_limits<double>::min();
    for (int i = 0; i < num_particles; i++) {
        weights.push_back(particles[i].weight);
        if (particles[i].weight > maxWeight) {
            maxWeight = particles[i].weight;
        }
    }

    // distributions.
    std::uniform_real_distribution<double> distDouble(0.0, maxWeight);
    std::uniform_int_distribution<int> distInt(0, num_particles - 1);

    int index = distInt(gen);
    double beta = 0.0;

    vector<Particle> resampledParticles;
    for (int i = 0; i < num_particles; i++) {
        beta += distDouble(gen) * 2.0;
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        resampledParticles.push_back(particles[index]);
    }

    particles = resampledParticles;
}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y) {

    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
    vector<double> v;

    if (coord == "X") {
        v = best.sense_x;
    } else {
        v = best.sense_y;
    }

    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}