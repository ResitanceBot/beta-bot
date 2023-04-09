//
// Created by Vasily Yuryev on 21.06.2021.
//

#pragma once
#include <cmath>
#include <string>
#include <vector>
#include <random>

namespace beacon_gazebo_sim {
    class RSSINoise {
    public:
        RSSINoise();
        RSSINoise(double m_rssi);
        double getRSSI(double distance, double time);
    private:
        double m_rssi;

//        double n1_update_p =


        double prev_time;
        std::default_random_engine random_generator;

        double laplace(double mean, double scale);
    };
}
