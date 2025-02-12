#include "seplos_bms.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace seplos_bms {

static const char *const TAG = "seplos_bms";

// 协议版本定义
#define SEPLOS_PROTOCOL_V21 0x21
#define SEPLOS_PROTOCOL_V25 0x25

// 字段偏移量配置（根据你的数据样本修正）
typedef struct {
  uint8_t cell_count_offset;       // 电池数量字段偏移
  uint8_t cell_voltages_start;     // 电池电压起始偏移
  uint8_t temp_sensor_count_offset;// 温度传感器数量偏移
  uint8_t temp_sensors_start;      // 温度传感器起始偏移
  uint8_t current_offset;          // 电流字段偏移
  uint8_t total_voltage_offset;    // 总电压字段偏移
  uint8_t residual_capacity_offset;// 剩余容量偏移
  uint8_t battery_capacity_offset; // 电池容量偏移
  uint8_t soc_offset;              // SOC偏移
  uint8_t rated_capacity_offset;   // 额定容量偏移
  uint8_t cycles_offset;           // 循环次数偏移
  uint8_t soh_offset;              // SOH偏移
  uint8_t port_voltage_offset;     // 端口电压偏移
} seplos_offsets_t;

// 协议版本字段映射表（修正后的 2.5 版本偏移量）
static const std::map<uint8_t, seplos_offsets_t> OFFSET_MAP = {
    {SEPLOS_PROTOCOL_V21,
     {
         .cell_count_offset = 7,
         .cell_voltages_start = 8,
         .temp_sensor_count_offset = 38,
         .temp_sensors_start = 39,
         .current_offset = 53,
         .total_voltage_offset = 55,
         .residual_capacity_offset = 57,
         .battery_capacity_offset = 61,
         .soc_offset = 63,
         .rated_capacity_offset = 65,
         .cycles_offset = 67,
         .soh_offset = 69,
         .port_voltage_offset = 71,
     }},
    {SEPLOS_PROTOCOL_V25,  // 根据你的数据样本修正
     {
         .cell_count_offset = 8,          // 数据样本中第8字节是 0x0F (15 cells)
         .cell_voltages_start = 9,        // 电压从第9字节开始 (0x0CF9)
         .temp_sensor_count_offset = 39,   // 第39字节是温度数量 0x06
         .temp_sensors_start = 40,        // 温度从第40字节开始
         .current_offset = 52,            // 电流在第52-53字节 (0x005C)
         .total_voltage_offset = 54,       // 总电压在54-55字节 (0xC27B)
         .residual_capacity_offset = 56,   // 剩余容量在56-57字节 (0x0801)
         .battery_capacity_offset = 60,    // 电池容量在60-61字节 (0x0028)
         .soc_offset = 62,                 // SOC在62-63字节 (0x0BB8)
         .rated_capacity_offset = 64,      // 额定容量在64-65字节 (未在样本中)
         .cycles_offset = 66,              // 循环次数在66-67字节 (未在样本中)
         .soh_offset = 68,                 // SOH在68-69字节 (未在样本中)
         .port_voltage_offset = 70,        // 端口电压在70-71字节 (0xE35A)
     }}};

void SeplosBms::on_seplos_modbus_data(const std::vector<uint8_t> &data) {
  if (data.size() < 8) {
    ESP_LOGE(TAG, "Invalid data length: %zu", data.size());
    return;
  }

  // 协议版本检测
  const uint8_t protocol_version = data[0];
  if (OFFSET_MAP.find(protocol_version) == OFFSET_MAP.end()) {
    ESP_LOGW(TAG, "Unsupported protocol version: 0x%02X", protocol_version);
    return;
  }

  this->on_telemetry_data_(data);
}

void SeplosBms::on_telemetry_data_(const std::vector<uint8_t> &data) {
  auto seplos_get_16bit = [&](size_t i) -> uint16_t {
    return (uint16_t(data[i]) << 8) | (uint16_t(data[i + 1]) << 0); // 修复括号错误
  };

  const uint8_t protocol_version = data[0];
  const seplos_offsets_t &offsets = OFFSET_MAP.at(protocol_version);

  ESP_LOGI(TAG, "Telemetry frame v%d.%d (%zu bytes)", protocol_version >> 4, protocol_version & 0x0F, data.size());
  ESP_LOGVV(TAG, "  %s", format_hex_pretty(data.data(), data.size()).c_str());

  // 解析电池信息
  uint8_t cells = (this->override_cell_count_) ? this->override_cell_count_ : data[offsets.cell_count_offset];
  ESP_LOGV(TAG, "Number of cells: %d", cells);

  float min_cell_voltage = 100.0f;
  float max_cell_voltage = -100.0f;
  float average_cell_voltage = 0.0f;
  uint8_t min_voltage_cell = 0;
  uint8_t max_voltage_cell = 0;

  // 解析电池电压（根据你的数据样本）
  for (uint8_t i = 0; i < std::min((uint8_t) 16, cells); i++) {
    const size_t pos = offsets.cell_voltages_start + (i * 2);
    if (pos + 1 >= data.size()) break; // 确保不越界

    uint16_t raw_voltage = seplos_get_16bit(pos);
    float cell_voltage = raw_voltage * 0.001f;
    average_cell_voltage += cell_voltage;

    // 调试输出原始数据
    ESP_LOGVV(TAG, "Cell %d raw: 0x%04X, voltage: %.3f V", i+1, raw_voltage, cell_voltage);

    if (cell_voltage < min_cell_voltage) {
      min_cell_voltage = cell_voltage;
      min_voltage_cell = i + 1;
    }
    if (cell_voltage > max_cell_voltage) {
      max_cell_voltage = cell_voltage;
      max_voltage_cell = i + 1;
    }
    this->publish_state_(this->cells_[i].cell_voltage_sensor_, cell_voltage);
  }
  average_cell_voltage /= cells;

  // 发布统计电压
  this->publish_state_(this->min_cell_voltage_sensor_, min_cell_voltage);
  this->publish_state_(this->max_cell_voltage_sensor_, max_cell_voltage);
  this->publish_state_(this->min_voltage_cell_sensor_, (float) min_voltage_cell);
  this->publish_state_(this->max_voltage_cell_sensor_, (float) max_voltage_cell);
  this->publish_state_(this->delta_cell_voltage_sensor_, max_cell_voltage - min_cell_voltage);
  this->publish_state_(this->average_cell_voltage_sensor_, average_cell_voltage);

  // 解析温度传感器
  uint8_t offset = offsets.temp_sensor_count_offset;
  if (offset >= data.size()) return;

  uint8_t temperature_sensors = data[offset];
  ESP_LOGV(TAG, "Temperature sensors: %d", temperature_sensors);

  for (uint8_t i = 0; i < std::min((uint8_t) 6, temperature_sensors); i++) {
    const size_t pos = offsets.temp_sensors_start + (i * 2);
    if (pos + 1 >= data.size()) break;

    uint16_t raw_temp = seplos_get_16bit(pos);
    float temperature = (raw_temp - 2731.0f) * 0.1f;
    ESP_LOGVV(TAG, "Temp %d raw: 0x%04X, value: %.1f C", i+1, raw_temp, temperature);
    this->publish_state_(this->temperatures_[i].temperature_sensor_, temperature);
  }

  // 解析电流和总电压（关键修正部分）
  if (offsets.current_offset + 1 < data.size()) {
    // 电流处理（有符号16位）
    int16_t raw_current = (int16_t)seplos_get_16bit(offsets.current_offset);
    float current = raw_current * 0.01f;
    ESP_LOGV(TAG, "Current raw: 0x%04X (%d), value: %.2f A", 
            raw_current, raw_current, current);
    this->publish_state_(this->current_sensor_, current);

    // 总电压处理
    if (offsets.total_voltage_offset + 1 < data.size()) {
      uint16_t raw_voltage = seplos_get_16bit(offsets.total_voltage_offset);
      float total_voltage = 0.00;
      if(protocol_version == SEPLOS_PROTOCOL_V21){
        total_voltage = raw_voltage * 0.01f
      }else{
        total_voltage = raw_voltage * 0.001f
      }
      ESP_LOGV(TAG, "Total voltage raw: 0x%04X, value: %.2f V", raw_voltage, total_voltage);
      this->publish_state_(this->total_voltage_sensor_, total_voltage);

      // 计算功率
      float power = total_voltage * current;
      this->publish_state_(this->power_sensor_, power);
      this->publish_state_(this->charging_power_sensor_, std::max(0.0f, power));
      this->publish_state_(this->discharging_power_sensor_, std::abs(std::min(0.0f, power)));
    }
  }

  // 安全发布数据的lambda（添加调试日志）
  auto safe_publish = [&](sensor::Sensor *sensor, size_t pos, float coeff, const char* name) {
    if (pos + 1 < data.size()) {
      uint16_t raw = seplos_get_16bit(pos);
      float value = raw * coeff;
      ESP_LOGV(TAG, "%s raw: 0x%04X, value: %.2f", name, raw, value);
      this->publish_state_(sensor, value);
    }
  };

  // 解析其他参数
  safe_publish(this->residual_capacity_sensor_, offsets.residual_capacity_offset, 0.01f, "Residual Capacity");
  safe_publish(this->battery_capacity_sensor_, offsets.battery_capacity_offset, 0.01f, "Battery Capacity");
  safe_publish(this->state_of_charge_sensor_, offsets.soc_offset, 0.1f, "SOC");
  safe_publish(this->rated_capacity_sensor_, offsets.rated_capacity_offset, 0.01f, "Rated Capacity");
  safe_publish(this->charging_cycles_sensor_, offsets.cycles_offset, 1.0f, "Cycles");
  safe_publish(this->state_of_health_sensor_, offsets.soh_offset, 0.1f, "SOH");
  safe_publish(this->port_voltage_sensor_, offsets.port_voltage_offset, 0.01f, "Port Voltage");
}

void SeplosBms::dump_config() {
  ESP_LOGCONFIG(TAG, "SeplosBms:");
  LOG_SENSOR("", "Minimum Cell Voltage", this->min_cell_voltage_sensor_);
  LOG_SENSOR("", "Maximum Cell Voltage", this->max_cell_voltage_sensor_);
  LOG_SENSOR("", "Minimum Voltage Cell", this->min_voltage_cell_sensor_);
  LOG_SENSOR("", "Maximum Voltage Cell", this->max_voltage_cell_sensor_);
  LOG_SENSOR("", "Delta Cell Voltage", this->delta_cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 1", this->cells_[0].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 2", this->cells_[1].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 3", this->cells_[2].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 4", this->cells_[3].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 5", this->cells_[4].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 6", this->cells_[5].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 7", this->cells_[6].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 8", this->cells_[7].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 9", this->cells_[8].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 10", this->cells_[9].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 11", this->cells_[10].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 12", this->cells_[11].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 13", this->cells_[12].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 14", this->cells_[13].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 15", this->cells_[14].cell_voltage_sensor_);
  LOG_SENSOR("", "Cell Voltage 16", this->cells_[15].cell_voltage_sensor_);
  LOG_SENSOR("", "Temperature 1", this->temperatures_[0].temperature_sensor_);
  LOG_SENSOR("", "Temperature 2", this->temperatures_[1].temperature_sensor_);
  LOG_SENSOR("", "Temperature 3", this->temperatures_[2].temperature_sensor_);
  LOG_SENSOR("", "Temperature 4", this->temperatures_[3].temperature_sensor_);
  LOG_SENSOR("", "Temperature 5", this->temperatures_[4].temperature_sensor_);
  LOG_SENSOR("", "Temperature 6", this->temperatures_[5].temperature_sensor_);
  LOG_SENSOR("", "Temperature 7", this->temperatures_[6].temperature_sensor_);
  LOG_SENSOR("", "Total Voltage", this->total_voltage_sensor_);
  LOG_SENSOR("", "Current", this->current_sensor_);
  LOG_SENSOR("", "Power", this->power_sensor_);
  LOG_SENSOR("", "Charging Power", this->charging_power_sensor_);
  LOG_SENSOR("", "Discharging Power", this->discharging_power_sensor_);
  LOG_SENSOR("", "Charging cycles", this->charging_cycles_sensor_);
  LOG_SENSOR("", "State of charge", this->state_of_charge_sensor_);
  LOG_SENSOR("", "Residual capacity", this->residual_capacity_sensor_);
  LOG_SENSOR("", "Battery capacity", this->battery_capacity_sensor_);
  LOG_SENSOR("", "Rated capacity", this->rated_capacity_sensor_);
  LOG_SENSOR("", "Charging cycles", this->charging_cycles_sensor_);
  LOG_SENSOR("", "State of health", this->state_of_health_sensor_);
  LOG_SENSOR("", "Port Voltage", this->port_voltage_sensor_);
}

float SeplosBms::get_setup_priority() const {
  // After UART bus
  return setup_priority::BUS - 1.0f;
}

void SeplosBms::update() { this->send(0x42, this->pack_); }

void SeplosBms::publish_state_(binary_sensor::BinarySensor *binary_sensor, const bool &state) {
  if (binary_sensor == nullptr)
    return;

  binary_sensor->publish_state(state);
}

void SeplosBms::publish_state_(sensor::Sensor *sensor, float value) {
  if (sensor == nullptr)
    return;

  sensor->publish_state(value);
}

void SeplosBms::publish_state_(text_sensor::TextSensor *text_sensor, const std::string &state) {
  if (text_sensor == nullptr)
    return;

  text_sensor->publish_state(state);
}

}  // namespace seplos_bms
}  // namespace esphome
