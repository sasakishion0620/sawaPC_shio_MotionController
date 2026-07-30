#ifndef MOTIONCONTROL_CONTEC_AD_H
#define MOTIONCONTROL_CONTEC_AD_H

#include <sys/io.h>
#include <cstdio>
#include <iomanip>
#include "adreader.h"
#include "json_helper.h"
#include "pci_helper.h"

struct contec_ad_device_set{
  unsigned int base0;
};

template<typename T>
class contec_ad : public adreader<T>
{
public:
  const int SUCCESS = 0;
  const int FAIL = 1;

  contec_ad(size_t start_channel, size_t number_of_channel, robot_system *robot, const char *config_file_name)
  : adreader<T>(start_channel, number_of_channel, robot),
    config_file_name_(config_file_name)
  {
    ad_resolution_decimal_ = NBitInDecimal(ad_resolution_bit_) - 1;
    open();
    std::cout << "contec ad constructor" << std::endl;
  }

  int open()
  {
    boost::property_tree::ptree pt;
    read_json(config_file_name_.c_str(), pt);

    vendor_id_numeric_ = static_cast<unsigned int>(
      strtol(json_helper<std::string>::get_value_from_json("vendor_id", pt).c_str(), NULL, 16));
    device_id_numeric_ = static_cast<unsigned int>(
      strtol(json_helper<std::string>::get_value_from_json("device_id", pt).c_str(), NULL, 16));
    board_order_ = json_helper<int>::get_value_from_json("board_order", pt);
    voltage_range_ = json_helper<double>::get_value_from_json("range", pt);
    ad_pci_.base0 = pci_helper::get_io_port_from_vendor_and_device_id(
      vendor_id_numeric_,
      device_id_numeric_,
      board_order_);

    std::cout << "[adreader] vendor_id: " << vendor_id_numeric_ << std::endl;
    std::cout << "[adreader] device_id: " << device_id_numeric_ << std::endl;
    std::cout << "[adreader] board_order: " << board_order_ << std::endl;
    std::cout << "[adreader] voltage_range: " << voltage_range_ << std::endl;
    std::cout << "[adreader] io port: " << ad_pci_.base0 << std::endl;
    print_config_summary();

    initialize();
    return SUCCESS;
  }

  int adread()
  {
    static int debug_count = 0;
    if (debug_count < 20)
    {
      std::printf("[contec_ad] entered adread()\n");
    }
    adreader<T>::adread();
    if (debug_count < 20)
    {
      std::printf("[contec_ad] leaving adread()\n");
      debug_count++;
    }
    return SUCCESS;
  }

  int scan()
  {
    static int debug_count = 0;
    unsigned int data = 0;

    if (debug_count < 20)
    {
      std::printf("[contec_ad] scan start, channels=%zu, base0=%u\n",
        adreader<T>::number_of_channel_,
        ad_pci_.base0);
      print_status_snapshot("before start");
    }

    outw(static_cast<unsigned int>(adreader<T>::number_of_channel_) - 1, ad_pci_.base0 + 0x02);
    if (debug_count < 20)
    {
      print_status_snapshot("after start command");
      print_register_window("after start command");
    }
    if (!test_busy_status(ad_pci_.base0))
    {
      std::printf("[contec_ad] busy timeout at base0=%u\n", ad_pci_.base0);
      print_status_snapshot("busy timeout");
      print_register_window("busy timeout");
      return FAIL;
    }

    if (debug_count < 20)
    {
      print_status_snapshot("after busy clear");
    }

    for (size_t ch = 0; ch < adreader<T>::number_of_channel_; ++ch)
    {
      data = inw(ad_pci_.base0);
      *(adreader<T>::channel_ptr(ch)) =
        static_cast<double>(data) / ad_resolution_decimal_ * voltage_range_ - voltage_range_ / 2;

      if (debug_count < 20)
      {
        std::printf("[contec_ad] ch=%zu raw=%u volt=%lf\n",
          ch,
          data,
          *(adreader<T>::channel_ptr(ch)));
      }
    }

    if (debug_count < 20)
    {
      debug_count++;
    }

    return adreader<T>::number_of_channel_;
  }

  void reset()
  {
    initialize_ad_board(ad_pci_.base0);
    setting_ad_board(ad_pci_.base0);
  }

  void initialize()
  {
    std::cout << "started contec ad board" << std::endl;
    iopl(3);
    print_diagnostic_header();
    print_status_snapshot("before reset");
    print_register_window("before reset");
    reset();
    print_status_snapshot("after reset");
    print_register_window("after reset");
    std::cout << "AD board reset" << std::endl;
  }

  void close()
  {
    std::cout << "contec ad is closed" << std::endl;
    reset();
  }

private:
  T ad_resolution_decimal_;
  const int ad_resolution_bit_ = 16;
  T voltage_range_;
  contec_ad_device_set ad_pci_;
  std::string config_file_name_;
  std::string vendor_id_;
  std::string device_id_;
  unsigned int vendor_id_numeric_ = 0;
  unsigned int device_id_numeric_ = 0;
  int board_order_ = 0;

  T NBitInDecimal(int power)
  {
    int out = 1;
    for (int i = 0; i < power; ++i)
    {
      out *= 2.0;
    }
    return out;
  }

  void setting_ad_board(int addr)
  {
    std::printf("[contec_ad] setting_ad_board addr=%d channels=%zu\n",
      addr,
      adreader<T>::number_of_channel_);
    print_register_plan();

    outw(0x00, addr + 0x06);
    outw(0x80, addr + 0x07);
    outw(0x00, addr + 0x07);

    for (int i = 0; i < static_cast<int>(adreader<T>::number_of_channel_); ++i)
    {
      outw(0x02, addr + 0x06);
      outw(static_cast<unsigned int>(i), addr + 0x07);
      outw(static_cast<unsigned int>(i), addr + 0x07);
    }

    outw(0x03, addr + 0x06);
    outw(0x27, addr + 0x07);
    outw(0x00, addr + 0x07);

    outw(0x04, addr + 0x06);
    outw(0x7f, addr + 0x07);
    outw(0x02, addr + 0x07);
    outw(0x00, addr + 0x07);
    outw(0x00, addr + 0x07);

    std::cout << "AD Start" << std::endl;
    print_status_snapshot("after setting");
    print_register_window("after setting");
  }

  void initialize_ad_board(int addr)
  {
    std::printf("[contec_ad] initialize_ad_board addr=%d\n", addr);
    outw(0x16, addr + 6);
  }

  bool test_busy_status(int addr)
  {
    int busy_sts;
    int guard = 1000000;
    int loop_count = 0;
    do
    {
      busy_sts = inw(addr + 0x02) & 1;
      if (loop_count < 5)
      {
        std::printf("[contec_ad] busy poll %d raw_status=%d busy=%d\n",
          loop_count,
          inw(addr + 0x02),
          busy_sts);
      }
      loop_count++;
      guard--;
      if (guard <= 0)
      {
        std::printf("[contec_ad] busy_sts stuck, addr=%d status_reg=%d\n", addr, inw(addr + 0x02));
        print_busy_interpretation();
        return false;
      }
    }
    while (busy_sts);
    return true;
  }

  void print_status_snapshot(const char *label)
  {
    const int base = static_cast<int>(ad_pci_.base0);
    std::printf(
      "[contec_ad] %s: reg+0x00=%d reg+0x02=%d reg+0x06=%d reg+0x07=%d\n",
      label,
      inw(base + 0x00),
      inw(base + 0x02),
      inw(base + 0x06),
      inw(base + 0x07));
  }

  void print_register_window(const char *label)
  {
    static int dump_count = 0;
    if (dump_count >= 12)
      return;

    const int base = static_cast<int>(ad_pci_.base0);
    std::printf("[contec_ad] %s register window:\n", label);
    for (int offset = 0; offset <= 0x1E; offset += 2)
    {
      const unsigned int word_value = static_cast<unsigned int>(inw(base + offset)) & 0xFFFF;
      std::printf("  reg+0x%02X = %5d (0x%04X)%s\n",
        offset,
        inw(base + offset),
        word_value,
        word_value == 0xFFFF ? "  <-- all bits 1" : "");
    }
    dump_count++;
  }

  void print_config_summary() const
  {
    std::cout << "[contec_ad] config summary: vendor=0x"
              << std::hex << vendor_id_numeric_
              << " device=0x" << device_id_numeric_
              << std::dec
              << " board_order=" << board_order_
              << " range=" << voltage_range_
              << " channels=" << adreader<T>::number_of_channel_
              << " start_channel=" << adreader<T>::start_channel_
              << std::endl;
  }

  void print_diagnostic_header() const
  {
    std::printf("\n");
    std::printf("========== AD Diagnostic ==========\n");
    std::printf("1) PCI candidate\n");
    std::printf("   vendor=0x%04X device=0x%04X board_order=%d\n",
      vendor_id_numeric_,
      device_id_numeric_,
      board_order_);
    std::printf("2) Selected I/O base\n");
    std::printf("   base0=0x%04X (%u)\n", ad_pci_.base0, ad_pci_.base0);
    std::printf("3) Current assumption\n");
    std::printf("   data register   : base+0x00\n");
    std::printf("   busy/start reg  : base+0x02\n");
    std::printf("   setting index   : base+0x06\n");
    std::printf("   setting data    : base+0x07\n");
    std::printf("4) What to watch\n");
    std::printf("   if many registers stay 0xFFFF, the address or offsets may be wrong\n");
    std::printf("===================================\n");
  }

  void print_register_plan() const
  {
    std::printf("[contec_ad] register plan:\n");
    std::printf("  initialize : outw(0x0016, base+0x06)\n");
    std::printf("  function   : outw(0x0000, base+0x06), then writes to base+0x07\n");
    std::printf("  channels   : outw(0x0002, base+0x06), then channel numbers to base+0x07\n");
    std::printf("  scan clock : outw(0x0003, base+0x06), then 0x27, 0x00 to base+0x07\n");
    std::printf("  samp clock : outw(0x0004, base+0x06), then 0x7f, 0x02, 0x00, 0x00 to base+0x07\n");
    std::printf("  scan start : outw(channel_count-1, base+0x02)\n");
    std::printf("  busy check : inw(base+0x02) & 1\n");
    std::printf("  data read  : inw(base+0x00)\n");
  }

  void print_busy_interpretation() const
  {
    std::printf("[contec_ad] busy interpretation:\n");
    std::printf("  if busy reg keeps reading 0xFFFF, we may be using the wrong register offset\n");
    std::printf("  if start command changes nothing, init/start sequence may not match this board\n");
    std::printf("  if base0 is wrong, every register can look dead even though PCI device was found\n");
  }
};

#endif // MOTIONCONTROL_CONTEC_AD_H
