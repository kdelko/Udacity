#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	if (!is_initialized)
	{
		num_particles=100;
		default_random_engine gen;
		normal_distribution<double> dist_x(x,std[0]);
		normal_distribution<double> dist_y(y,std[1]);
		normal_distribution<double> dist_theta(theta,std[2]);

		particles.resize(num_particles);
		weights.resize(num_particles);

		for (int i=0;i<num_particles;i++)
		{
			Particle p;
			p.id = i;
			p.x=dist_x(gen);
			p.y=dist_y(gen);
			p.theta=dist_theta(gen);
			p.weight=1.0;

			weights[i]=(p.weight);
			particles[i]=p;
		}
		is_initialized=true;
	}

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;

	normal_distribution<double> pos_x(0,std_pos[0]);
	normal_distribution<double> pos_y(0,std_pos[1]);
	normal_distribution<double> pos_theta(0,std_pos[2]);

	for (int i=0;i<num_particles;i++)
	{
		double new_x, new_y, new_theta;

		if (fabs(yaw_rate)<0.0001) //just needs to be a small number
		{
			new_x=particles[i].x+velocity*delta_t*cos(particles[i].theta);
			new_y=particles[i].y+velocity*delta_t*sin(particles[i].theta);
			new_theta=particles[i].theta;
		}
		else
		{
			new_x=particles[i].x+(velocity/yaw_rate)*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta));
			new_y=particles[i].y+(velocity/yaw_rate)*(cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t));
			new_theta=particles[i].theta+yaw_rate*delta_t;
		}

		particles[i].x=new_x+pos_x(gen);
		particles[i].y=new_y+pos_y(gen);
		particles[i].theta=new_theta+pos_theta(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.


}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html



	//vector<LandmarkObs> assoc_landmarks;
	double arg1;
	double arg2;

	for (int i=0;i<num_particles;i++)
	{

		vector<LandmarkObs> range_lm;
		for (int k=0;k<map_landmarks.landmark_list.size();k++)
		{
			Map::single_landmark_s m_landmark = map_landmarks.landmark_list[k];
			double distance=dist(m_landmark.x_f,m_landmark.y_f,particles[i].x,particles[i].y);
			if (distance<sensor_range)
			{
				range_lm.push_back(LandmarkObs{m_landmark.id_i, m_landmark.x_f, m_landmark.y_f});
			}

		}

		vector<LandmarkObs> observations_m;
		LandmarkObs ob_m;

		for (int l=0;l<observations.size();l++)
		{

			ob_m.x=particles[i].x+cos(particles[i].theta)*observations[l].x-sin(particles[i].theta)*observations[l].y;//transform coordinate system
			ob_m.y=particles[i].y+sin(particles[i].theta)*observations[l].x+cos(particles[i].theta)*observations[l].y;


			double distance,temp_dist;
			int map_idx =-1;
 			distance = numeric_limits<double>::max();
			for (int j=0;j<range_lm.size();j++)
			{
				temp_dist=dist(ob_m.x,ob_m.y,range_lm[j].x,range_lm[j].y);
				if (temp_dist<distance)
				{
					distance=temp_dist;
					map_idx=range_lm[j].id;
				}
			}
			ob_m.id=map_idx;
			observations_m.push_back(ob_m);
		}


		particles[i].weight=1.0;//initialize particle filter

		for (int l=0;l<observations_m.size();l++)
		{
			LandmarkObs landmark_nearest;
			for (int j=0;j<range_lm.size();j++)
			{
				if (range_lm[j].id==observations_m[l].id)
					{
						landmark_nearest.id=range_lm[j].id;
						landmark_nearest.x=range_lm[j].x;
						landmark_nearest.y=range_lm[j].y;
					}
			}

			arg1=observations_m[l].x-landmark_nearest.x;
			arg2=observations_m[l].y-landmark_nearest.y;

			cout<<particles[i].weight<<endl;
			particles[i].weight*=1/(2*M_PI*std_landmark[0]*std_landmark[1])*
			exp(-((arg1*arg1)/(2*std_landmark[0]*std_landmark[0])+(arg2*arg2)/
				(2*std_landmark[1]*std_landmark[1])));//this is the forumla to calucalte the particle weights according to
				//lesson segment 19 in lesson 15. Normalizer * exponent


			//cout<<"Obs: "<<observations[l].x<<","<<observations[l].y<<"Particle: "<<particles[i].x
			//<<","<<particles[i].y<<","<<particles[i].theta<<" Obs in Map: "<<
			//observations_m[l].x<<","<<observations_m[l].y<<" Near Landmark: "<<landmark_nearest.x<<","<<landmark_nearest.y<<endl;
		}
		weights[i]=particles[i].weight;
		//cout<<"particle weight calculated! --> "<<particles[i].weight<<endl;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	vector<Particle> new_particles;
	random_device seed;
	mt19937 random_generator(seed());

	discrete_distribution<> sample(weights.begin(), weights.end());

	for (int i = 0; i < num_particles; i++)
	{
		new_particles.push_back(particles[sample(random_generator)]);
	}
	particles = new_particles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
