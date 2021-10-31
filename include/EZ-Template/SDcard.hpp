#pragma once
#include  "EZ-Template/AutonSelector.hpp"
namespace EZ
{
  namespace SD
  {
    extern AutonSelector autoSelector;
    extern const bool IS_SD_CARD;
    void update_auto_sd();
    void init_auto_sd();
    void page_up();
    void page_down();
  }
}
