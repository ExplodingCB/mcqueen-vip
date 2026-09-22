#include <iomanip>
#include <iostream>
#include <stdexcept>

#include "mcq_track/track.hpp"
int main(int argc, char ** argv)
{
  try {
    if (argc != 3) throw std::invalid_argument("usage: track_probe CSV closed");
    const auto track = mcq_track::Track::from_csv(argv[1], std::string(argv[2]) == "1");
    std::cout << std::setprecision(17);
    double s, d, x, y;
    while (std::cin >> s >> d >> x >> y) {
      const auto p = track.cartesian(s, d);
      const auto w = track.width_at(s), sd = track.frenet(x, y);
      std::cout << track.length() << ' ' << p.x << ' ' << p.y << ' ' << p.heading << ' '
                << track.curvature_at(s) << ' ' << w.first << ' ' << w.second << ' ' << sd.first
                << ' ' << sd.second << '\n';
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
