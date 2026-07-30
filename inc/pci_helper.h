#ifndef MOTIONCONTROL_PCI_HELPER_H
#define MOTIONCONTROL_PCI_HELPER_H

#include <iostream>
#include <stdexcept>
#include <vector>
#include <algorithm>
extern "C" {
#include <pci/pci.h>
}

class pci_helper
{
public:
  static unsigned int get_io_port_from_vendor_and_device_id(unsigned int vendor, unsigned int device, int order)
  {
    unsigned int io_port = 0;
    pci_access *p_access = nullptr;

    p_access = pci_alloc();
    pci_init(p_access);

    std::vector<pci_dev*> pci_devs = find_pci_dev(vendor, device, p_access);
    if (pci_devs.size() <= static_cast<size_t>(order))
    {
      std::cerr << "[Error]" << std::endl;
      std::cerr << "\tPCI device could not found by vendor id or device id or the order." << std::endl;
      pci_cleanup(p_access);
      throw std::runtime_error("PCI device not found: vendor/device/order mismatch");
    }
    std::sort(pci_devs.begin(), pci_devs.end(), [](pci_dev *a, pci_dev *b)
    {return a->bus < b->bus;});

    std::cout << "[pci_helper] matched devices for vendor=0x"
              << std::hex << vendor
              << " device=0x" << device
              << std::dec << std::endl;
    for (size_t i = 0; i < pci_devs.size(); ++i)
    {
      pci_dev *dev = pci_devs[i];
      std::cout << "[pci_helper] idx=" << i
                << " bus=" << static_cast<int>(dev->bus)
                << " dev=" << static_cast<int>(dev->dev)
                << " func=" << static_cast<int>(dev->func)
                << std::endl;
      for (int bar = 0; bar < 6; ++bar)
      {
        const unsigned int raw_bar = static_cast<unsigned int>(dev->base_addr[bar]);
        std::cout << "  [pci_helper] BAR" << bar
                  << "=0x" << std::hex << raw_bar
                  << " (" << describe_bar(raw_bar) << ")"
                  << std::dec << std::endl;
      }
    }

    const unsigned int selected_bar0 = static_cast<unsigned int>(pci_devs[order]->base_addr[0]);
    std::cout << "[pci_helper] selecting order=" << order
              << " using BAR0=0x" << std::hex << selected_bar0
              << " (" << describe_bar(selected_bar0) << ")"
              << std::dec << std::endl;
    if ((selected_bar0 & 0x1u) == 0)
    {
      std::cout << "[pci_helper] warning: BAR0 does not look like I/O space" << std::endl;
    }
    io_port = static_cast<unsigned int>(selected_bar0 - 1);
    std::cout << "[pci_helper] interpreted I/O port address = 0x"
              << std::hex << io_port
              << std::dec << " (" << io_port << ")" << std::endl;
    pci_cleanup(p_access);
    return io_port;
  }
private:
  static const char *describe_bar(unsigned int raw_bar)
  {
    if (raw_bar == 0)
      return "unused";
    if (raw_bar & 0x1u)
      return "I/O space";
    return "memory space";
  }

  static std::vector<pci_dev*> find_pci_dev(unsigned int vendor, unsigned int device, pci_access *p_access)
  {
    std::vector<pci_dev*> ret;

    pci_dev *p_dev = nullptr;

    pci_scan_bus(p_access);
    for (p_dev = p_access->devices; p_dev; p_dev = p_dev->next)
    {
      pci_fill_info(p_dev, PCI_FILL_IDENT | PCI_FILL_BASES);
      if (p_dev->vendor_id == vendor && p_dev->device_id == device)
        ret.push_back(p_dev);
      else
        continue;
    }
    return ret;
  }
};
#endif //MOTIONCONTROL_PCI_HELPER_H
