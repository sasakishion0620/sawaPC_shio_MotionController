#ifndef MOTIONCONTROL_CONTEC_AD_H
#define MOTIONCONTROL_CONTEC_AD_H

#include <sys/io.h>
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

    unsigned int vendor_id = static_cast<unsigned int>(
      strtol(json_helper<std::string>::get_value_from_json("vendor_id", pt).c_str(), NULL, 16));
    unsigned int device_id = static_cast<unsigned int>(
      strtol(json_helper<std::string>::get_value_from_json("device_id", pt).c_str(), NULL, 16));
    int board_order = json_helper<int>::get_value_from_json("board_order", pt);
    voltage_range_ = json_helper<double>::get_value_from_json("range", pt);
    ad_pci_.base0 = pci_helper::get_io_port_from_vendor_and_device_id(vendor_id, device_id, board_order);

    std::cout << "[adreader] vendor_id: " << vendor_id << std::endl;
    std::cout << "[adreader] device_id: " << device_id << std::endl;
    std::cout << "[adreader] io port: " << ad_pci_.base0 << std::endl;

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

    outw(static_cast<unsigned int>(adreader<T>::number_of_channel_) - 1, ad_pci_.base0 + 0x02);
    test_busy_status(ad_pci_.base0);

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
    reset();
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
  }

  void initialize_ad_board(int addr)
  {
    outw(0x16, addr + 6);
  }

  void test_busy_status(int addr)
  {
    int busy_sts;
    do
    {
      busy_sts = inw(addr + 0x02) & 1;
    }
    while (busy_sts);
  }
};

#endif // MOTIONCONTROL_CONTEC_AD_H
