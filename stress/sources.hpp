#ifndef STRESS_SOURCES_HPP_INCLUDED
#define STRESS_SOURCES_HPP_INCLUDED

#include "model/stress_base.hpp"


namespace model {
  namespace stress {

    template <typename Agent>
    class predator_distance
    {
      make_stress_source_from_this(predator_distance);

    public:
      predator_distance() = default;
      predator_distance(size_t, const json& J)
      {
        w_ = J["w"];
        shape_ = J["distr_shape"];
      }

      void operator()(agent_type* self, size_t idx, tick_t T, const Simulation& sim)
      {
        auto ip = sim.sorted_view<Tag, pred_tag>(idx);
        if (!ip.empty()) {
            self->stress += w_ * std::exp( - std::sqrt(ip[0].dist2) / shape_);
        }
      }

    private:
      float w_ = 0;             // [1] stress weight
      float shape_ = 0;   // shape of exponential function
    };
  }
}
#endif
