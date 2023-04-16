//
// Created by Vasily Yuryev on 22.06.2021.
//
#include <beacon_gazebo_sim/rssi_noise.h>

namespace beacon_gazebo_sim {
    RSSINoise::RSSINoise() {
        this->m_rssi = -60;
    }

    RSSINoise::RSSINoise(double m_rssi) {
        this->m_rssi = m_rssi;
    }

    double RSSINoise::laplace(double mean, double scale) {
        return (-scale*std::log(this->random_generator())) - (-scale*std::log(this->random_generator())) + mean;
    }

    double RSSINoise::getRSSI(double distance, double time) { // TODO: add noise simulation
        double a = 10;
        double b = 1;
        double c = 0;

        /*
         * Algorithm idea:
         *  generate base noise using laplace which will update with constant rate
         *  and next add small gaussian noise
         */

        double rssi_1 = this->m_rssi * std::pow(((distance - c) / b), (1.0 / a));


        double dt = time - this->prev_time;

        std::normal_distribution<double> nd_rssi(rssi_1, 0.6);
        double rssi_out = nd_rssi(this->random_generator);
//        double rssi_out = laplace(rssi_1, 2); // I think this is the closest distribution to the real one
        this->prev_time = time;
        return rssi_out;
    }
}
