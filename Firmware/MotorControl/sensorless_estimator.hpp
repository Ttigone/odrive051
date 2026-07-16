#ifndef __SENSORLESS_ESTIMATOR_HPP
#define __SENSORLESS_ESTIMATOR_HPP

// 无感估测
class SensorlessEstimator : public ODriveIntf::SensorlessEstimatorIntf {
 public:
  struct Config_t {
    float observer_gain = 1000.0f;     // [rad/s]
    float pll_bandwidth = 1000.0f;     // [rad/s]
    float pm_flux_linkage = 1.58e-3f;  // [V / (rad/s)]  { 5.51328895422 /
                                       // (<pole pairs> * <rpm/v>) }
  };

  // NOTE 构造函数使用非 const 引用 (Config_t& config) 的原因：
  // 1. 避免拷贝：直接传引用开销更低
  // 2. 配置联动：外部修改 config 时，SensorlessEstimator 内部同步变化，无需额外同步机制
  // 3. 生命周期由调用方管理：不获取所有权，但 SensorlessEstimator 的生命周期必须短于传入的 config
  //    （风险点：若 config 被提前销毁，会产生悬垂引用）
  explicit SensorlessEstimator(Config_t& config);

  bool update();

  Axis* axis_ = nullptr;  // set by Axis constructor
  Config_t& config_;

  // TODO: expose on protocol
  Error error_ = ERROR_NONE;
  float phase_ = 0.0f;              // [rad]
  float pll_pos_ = 0.0f;            // [rad]
  float vel_estimate_ = 0.0f;       // [turn/s]
  float vel_estimate_erad_ = 0.0f;  // [rad/s]
  bool vel_estimate_valid_ = false;
  // float pll_kp_ = 0.0f;                       // [rad/s / rad]
  // float pll_ki_ = 0.0f;                       // [(rad/s^2) / rad]
  float flux_state_[2] = {0.0f, 0.0f};           // [Vs]
  float V_alpha_beta_memory_[2] = {0.0f, 0.0f};  // [V]
  bool estimator_good_ = false;
};

#endif /* __SENSORLESS_ESTIMATOR_HPP */
