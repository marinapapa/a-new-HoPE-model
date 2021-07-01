//#include "agents/agents_fwd.hpp"
#include "model/init_cond.hpp"
#include "agents/agents.hpp"

namespace model {

  namespace {
    thread_local rndutils::mutable_discrete_distribution<int, rndutils::all_zero_policy_uni> pigeon_discrete_dist;
  }
  
  decltype(Pigeon::transitions_) Pigeon::transitions_;

  template <typename Init>
  void do_init_pop(std::vector<snapshot_entry<pigeon_tag>>& vse, Init&& init)
  {
    for (auto& e : vse) init(e);
  }

  std::vector<snapshot_entry<pigeon_tag>> Pigeon::init_pop(const Simulation& sim, const json& J)
  {
    const size_t N = J["N"];
    auto jic = J["InitCondit"];
    std::string type = jic["type"];
    if (type == "none") return {};
    std::vector<snapshot_entry<pigeon_tag>> vse(N);
    if (type == "random") do_init_pop(vse, initial_conditions::random_pos_dir(jic));
	  else if (type == "flock") do_init_pop(vse, initial_conditions::in_flock(jic));
    else throw std::runtime_error("unknown initializer");
    return vse;
  }


  Pigeon::Pigeon(size_t idx, const json& J) :
    current_state_(0),
    pos(0, 0),
    dir(1, 0),
    accel(0) // [m / s^2]
  {
   
    if (idx == 0) {
      transitions_ = decltype(transitions_)(J);
    }

    pa_ = AP::create(idx, J["states"]);
    float stress_mean = J["stress"]["ind_var_mean"];
    float stress_sd = J["stress"]["ind_var_sd"];
    if (stress_sd)
    {
      auto str_pdist = std::normal_distribution<float>(stress_mean, stress_sd);
      stress = stress_ofs_ = str_pdist(model::reng);
    }
    else { stress = stress_ofs_ = 0.f; }
    sp_ = stress_accum::create(idx, J["stress"]["sources"]);

    ai = flight::create_aero_info<float>(J["aero"]);
    sa.w = 0.f; // until they get value from state (first integrates before update)
    speed = sa.cruiseSpeed = ai.cruiseSpeed;

  }

  void Pigeon::initialize(size_t idx, const Simulation& sim, const json& J)
  {
    pa_[current_state_]->enter(this, idx, 0, sim);
  }

  ::model::instance_proxy Pigeon::instance_proxy(long long color_map, size_t idx, const Simulation* sim) const noexcept
  {
    float tex = -1.f;
    switch (color_map) {
    case 1: tex = float(idx) / sim->pop<Tag>().size(); break;
    case 2: tex = glm::clamp(speed / ai.maxSpeed, 0.f, 1.f); break;
    case 3: {
      tex = 0.5f + flight_control::bank(this) / math::pi<float>; break;
    }
    case 4: tex = float(current_state_) / AP::size; break;
    case 5: tex = float(sim->flock_of<Tag>(idx)) / sim->flocks<Tag>().size(); break;
    case 6: tex = float(am_target); 
    };
    tex = std::clamp(tex, -1.f, 1.f);  // yes -1,+1, need '-1' in shader
    return { pos, speed * dir, glmutils::perpDot(dir), tex };
  }

  ::model::snapshot_entry<pigeon_tag> Pigeon::snapshot(const Simulation* sim, size_t idx) const noexcept
  {
    return { pos, dir, speed , accel};
  }

  void Pigeon::snapshot(Simulation* sim, size_t idx, const snapshot_entry<pigeon_tag>& se) noexcept
  {
    pos = se.pos;
    speed = se.speed;
	  dir = se.dir; 
	  accel = se.accel;
    stress = se.stress;

  }

  size_t Pigeon::update(size_t idx, tick_t T, const Simulation& sim)
  {
    steering = vec_t(0); 
    am_target = false;
    pa_[current_state_]->resume(this, idx, T, sim);
    last_update = T;
    return T + reaction_time;
  }

  void Pigeon::integrate(tick_t T, const Simulation& sim)
  {
    flight_control::integrate_motion(this);    
  }

  void Pigeon::on_state_exit(size_t idx, tick_t T, const Simulation& sim)
  {
    // select new state & enter
    stress = stress_ofs_;
    stress_accum::apply(sp_, this, idx, T, sim);
    const auto TM = transitions_(stress);
    tm = TM[current_state_];
    pigeon_discrete_dist.mutate(TM[current_state_].cbegin(), TM[current_state_].cend());
    current_state_ = pigeon_discrete_dist(reng);
    pa_[current_state_]->enter(this, idx, T, sim);
  }
}
